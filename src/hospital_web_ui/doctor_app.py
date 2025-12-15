import streamlit as st
import gspread
from oauth2client.service_account import ServiceAccountCredentials
import pandas as pd
from datetime import datetime
import time

# [추가됨] ROS 2 통신을 위한 라이브러리
import rclpy
from std_msgs.msg import Bool

# ==========================================
# 0. ROS 2 신호 전송 함수 (새로 추가된 부분)
# ==========================================
def publish_next_signal():
    """
    진료 완료 버튼을 누르면 로봇에게 '다음으로 가라'는 신호를 보냅니다.
    Topic: /hospital/next_waypoint (Bool)
    """
    try:
        # ROS 2 초기화 (이미 되어있으면 패스)
        if not rclpy.ok():
            rclpy.init()

        # 충돌 방지를 위해 유니크한 노드 이름 생성
        node_name = f'doctor_ui_publisher_{int(time.time())}'
        node = rclpy.create_node(node_name)
        
        # Publisher 생성 (로봇 쪽 SmartDispatcher와 토픽 이름 일치!)
        pub = node.create_publisher(Bool, '/hospital/next_waypoint', 10)

        # 메시지 생성 및 발행 (True 신호 전송)
        msg = Bool()
        msg.data = True
        pub.publish(msg)
        
        # 메시지가 확실히 나가도록 아주 잠깐 대기
        time.sleep(0.1)
        
        # 정리 (노드 삭제 및 종료)
        node.destroy_node()
        rclpy.shutdown()
        
        return True, "로봇에게 이동 명령을 보냈습니다."
    except Exception as e:
        return False, f"ROS 통신 오류: {e}"

# ==========================================
# 1. 구글 시트 연결 설정
# ==========================================
def connect_google_sheet():
    scope = ["https://spreadsheets.google.com/feeds", "https://www.googleapis.com/auth/drive"]
    # 다운받은 json 파일 이름이 정확해야 합니다!
    creds = ServiceAccountCredentials.from_json_keyfile_name("service_account.json", scope)
    client = gspread.authorize(creds)
    # 구글 시트 파일 이름 (정확히 적어주세요)
    sheet = client.open("medical_records") 
    return sheet

# ==========================================
# 2. UI 구성 (Streamlit)
# ==========================================
st.set_page_config(page_title="🏥 병원 진료 시스템", layout="wide")

st.title("👨‍⚕️ 의사 전용 대시보드 (Doctor UI)")

try:
    # 시트 연결
    sheet_file = connect_google_sheet()
    
    # [설문지 응답 시트1]에서 환자 목록 가져오기
    # A열: patient_id, B열: 타임스탬프, C열: 이름, F열: 증상
    worksheet_patients = sheet_file.worksheet("설문지 응답 시트1")
    data = worksheet_patients.get_all_records() # 데이터를 딕셔너리 리스트로 가져옴
    df = pd.DataFrame(data)

    # 사이드바: 환자 선택
    st.sidebar.header("환자 대기 목록")
    
    # 환자 ID 선택 상자 (PAT-00xx)
    if not df.empty and 'patient_id' in df.columns:
        patient_list = df['patient_id'].tolist()
        selected_patient_id = st.sidebar.selectbox("진료할 환자를 선택하세요", patient_list)

        # 선택된 환자 정보 가져오기
        patient_info = df[df['patient_id'] == selected_patient_id].iloc[0]

        # --- 메인 화면: 환자 정보 표시 ---
        col1, col2 = st.columns(2)
        with col1:
            st.info(f"### 📋 환자 정보: {patient_info.get('이름', '이름없음')}")
            st.write(f"**ID:** {selected_patient_id}")
            st.write(f"**성별:** {patient_info.get('성별', '-')}")
            st.write(f"**나이:** {patient_info.get('나이', '-')}")
            st.write(f"**접수시간:** {patient_info.get('타임스탬프', '-')}")
        
        with col2:
            st.error(f"### 🚨 주요 증상")
            st.write(f"{patient_info.get('증상', '증상 정보 없음')}")

        st.markdown("---")

        # --- 진료 입력 폼 ---
        st.subheader("📝 진료 기록 작성")
        
        with st.form("diagnosis_form"):
            doctor_name = st.text_input("담당 의사", value="김닥터")
            diagnosis = st.text_area("진단 소견", placeholder="예: 급성 편도염")
            prescription = st.text_area("처방 내용", placeholder="예: 항생제 3일분, 충분한 휴식")
            
            # 제출 버튼
            submitted = st.form_submit_button("✅ 진료 완료 및 로봇 호출")

            if submitted:
                # [시트2]에 데이터 저장
                worksheet_records = sheet_file.worksheet("시트2")
                
                # 현재 시간
                now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                # 시트2의 다음 빈 행에 추가
                worksheet_records.append_row([
                    selected_patient_id, # A열
                    "내과",              # B열 (진료과 예시)
                    diagnosis,           # C열
                    "",                  # D열 (소견)
                    prescription,        # E열
                    doctor_name,         # F열
                    now_str,             # G열
                    True                 # H열 (체크박스 TRUE -> 이메일 자동 발송 트리거!)
                ])
                
                # ---------------------------------------------------------
                # [추가됨] 로봇에게 출발 신호 보내기 (ROS 2)
                # ---------------------------------------------------------
                with st.spinner("🤖 로봇에게 신호를 보내는 중..."):
                    success, msg = publish_next_signal()
                    if success:
                        st.info(f"📢 {msg}")
                    else:
                        st.warning(f"⚠️ {msg} (데이터 저장은 성공함)")
                # ---------------------------------------------------------

                st.success(f"{patient_info.get('이름')} 환자의 진료 기록이 저장되고, 이메일이 발송되었습니다!")
                st.balloons()
    
    else:
        st.warning("대기 중인 환자가 없거나 데이터베이스를 읽을 수 없습니다.")

except Exception as e:
    st.error(f"오류가 발생했습니다: {e}")
    st.write("팁: service_account.json 파일이 있는지, 시트 이름이 정확한지 확인하세요.")
