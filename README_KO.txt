PSFB–CLLLC 병렬 전류 인계 연구 | MATLAB R2024b 재구성 패키지
============================================================

[실행]
ZIP 압축을 해제한 폴더를 MATLAB Current Folder로 지정한다.
Command Window에 다음 한 줄을 실행한다.

    build_charger_models

현재 폴더 아래 PSFB_CLLLC_models 폴더에 6개의 .slx가 생성된다.
    PSFB_single_4ohm.slx
    PSFB_single_8ohm.slx
    CLLLC_single_8ohm.slx
    Parallel_Base.slx
    Case_A.slx
    Case_B.slx

첫 모델만 생성하려면:
    build_charger_models('PSFB_single_4ohm')

먼저 단독 회로를 시험한다. 5 ms는 컴파일 확인용이며 기동 전 구간이다.
    run_charger_models('PSFB_single_4ohm',0.005)
    run_charger_models('PSFB_single_4ohm',0.04)
    run_charger_models('CLLLC_single_8ohm',0.04)

정상상태를 포함하는 전체 실행:
    run_charger_models('PSFB_single_4ohm')
    run_charger_models('CLLLC_single_8ohm')

두 모듈 단독 구동과 초기 전류를 검증한 다음 병렬 실행:
    run_charger_models({'Case_A','Case_B'})

자동으로 PSFB_CLLLC_models/data에 CSV, 파형 PNG, 비교 지표 CSV를 생성한다.
시험 실행(StopTime을 별도 지정)은 파일명 뒤에 _test가 붙으므로 본실험 데이터를 덮어쓰지 않는다.
전체 0.3 s 실험은 50 ns 계산 간격(600만 step)으로 계산하므로 실행 시간이 길 수 있다.
모델 창에서 직접 Run 버튼으로 실행해도 된다. Scope로 파형을 확인할 수 있다.

[구현 범위]
- Simscape Electrical Specialized Power Systems 실제 스위칭 모델.
- 400 V 이상 DC 전압원, IGBT/다이오드 풀브리지, 고주파 변압기, 다이오드 정류기.
- PSFB: 10 kHz, 출력 RL 필터 및 전류 PI 위상천이 제어.
- CLLLC: 100 kHz 고정 주파수, Crp/Lrp/Lm/Lrs/Crs, 전류 PI 위상천이 제어(PFM 없음).
- 공통 4 ohm 부하, 모듈별 출력전류 센서, 공통 부하전류 및 전압 센서.
- Case A: 0.05~0.10 s에 PSFB 50→25 A 및 CLLLC 0→25 A 램프 인계.
- Case B: 같은 CLLLC 램프; PSFB 지령 = saturation(50 - filtered I_CLLLC, 0, 50).
- 초기 PSFB 지령은 0.01~0.03 s에 기동 램프, CLLLC는 인계 전 게이트 OFF.
- 두 제어의 공통 조건은 동일. 부하 저항이므로 I_load=V_out/4; I_total은 커패시터 전류를 포함할 수 있음.
- 제어기는 carrier 주기별 PI 갱신, 계산 간격에서 게이트 생성; 적분 제한 및 deadtime 포함.

[출처 및 가정]
기존 자료: 회로스크립트.txt, 결과값.docx 1~4쪽, 전력전자 Final Project, 앞서 첨부된 CLLLC 회로 및 파라미터 화면.
PSIM 원본 .psimsch 7개는 바이너리로, 최신 블록 내부 파라미터는 읽을 수 없었다.
따라서 본 모델은 원본을 직접 변환한 것이 아니라 확인된 구조를 재구성한 것이다.

확인 또는 이전 합의 기반: Vin 400 V, 목표 200 V, 4/8 ohm, PSFB 10 kHz, CLLLC 100 kHz,
PSFB 권선비 1:1, CLLLC 초기 권선비 2:1, PSFB 출력 L=2 mH, 단독 출력 C=1 mF,
CLLLC Crp=309 nF, Lrp=8.2 uH, Lm=41 uH, Lrs=2.05 uH, Crs=1.236 uF,
CLLLC 이전 변압기 설정 Rp/Rs=1 uohm, Lp/Ls=1 uH, deadtime 0.2 us.

재구성 가정: PFC 제외(팀원 최신 PSIM 결과는 PFC가 포함되어 있어 입력 조건 다름),
최신 결과 보고서의 램프 인계 채택(초기 txt는 Case A step 지령),
공통 Cbus=2 mF(두 1 mF의 합으로 가정), PSFB 내부 누설 값은 CLLLC 이전 설정과 같은 초기값,
반도체 Ron=1 mohm, Vf=0, 저항형 스너버 100 kohm, 출력 ESR=1 mohm, 공진 직렬 R=5 mohm,
PSFB deadtime=1 us, PI 게인 및 전류 측정 필터는 초기 튜닝값(최신 팀원 회로 값과 일치 보장 없음).
SPS 변압기의 내부 자화 L=inf, Rm=100 Mohm, CLLLC의 자화 L은 외부 41 uH로 모델링.
입력과 출력 기준점 사이 1 Gohm 수치적 기준 저항은 계산용으로 추가.
반도체 상세 스위칭 손실, 열, 실제 배터리 전기화학 모델은 포함하지 않음.

[수정과 검증]
빌드 파일 defaults()에 모든 설계값이 있다. 또는 overrides 구조체로 수정 가능.
예: build_charger_models('CLLLC_single_8ohm',struct('nC',1.8))
위 예는 권선비 조정 시험이며 200 V 도달을 보장하지 않는다.
CLLLC 2:1에서는 최대 위상천이로도 부하 전압/전류를 달성하기 어려울 수 있다.
CLLLC 단독 25 A, PSFB 단독 50 A가 달성되지 않으면 게인, 권선비, 공진소자 등을 검증 후 조정한다.
최종 비교에서는 두 Case에 반드시 동일 파라미터를 적용한다.
생성된 모델을 열어 수정했다면 다시 생성하기 전 저장하고 닫을 것. 기존 .slx는 .bak으로 백업한다.
0.05 s 이전 PSFB 50 A, CLLLC 0 A, Vout 200 V 조건을 만족하는지 확인한다.
필요하면 tEnable을 늦추고 t0/t1 분석 범위도 함께 변경한다.
예: compare_charger_cases([],0.10,0.20)
비교 지표는 100 us causal moving average 기준으로 산출, 최대 raw 오차도 별도로 기록한다.
정착시간은 인계 이후 ±1 A 범위에 시뮬레이션 종료까지 머무는 시각으로 정의(미정착은 NaN).
모델에 실측 데이터가 들어 있지 않으므로 예측 기반 제어로 부르지 말 것.

[중요한 실행 한계]
본 패키지는 MATLAB R2024b 공식 블록 문서를 기준으로 작성했으나, 제작 환경에 MATLAB/Simulink가 없어
실제 .slx 생성, 컴파일, 라이선스 사용 및 전력회로 수렴 검증을 수행하지 못했다.
실행 오류가 발생하면 MATLAB Command Window의 오류 전문과 해당 블록을 확인한다.
라이브러리 블록 경로는 sps_lib 및 기존 powerlib 경로를 탐색하고, 마스크는 이름/표시 문구로 찾는다.
원본 PSIM과 정량적으로 일치한다는 주장은 검증 이후에만 가능하다.

[MathWorks 참고]
https://www.mathworks.com/help/releases/R2024b/sps/powersys/ref/universalbridge.html
https://www.mathworks.com/help/releases/R2024b/sps/powersys/ref/lineartransformer.html
https://www.mathworks.com/help/releases/R2024b/sps/powersys/ref/seriesrlcbranch.html
https://www.mathworks.com/help/releases/R2024b/simulink/slref/toworkspace.html
