README FILE

# git 규칙
1. upload는 parking.py만 하시면 됩니다.
2. 각자의 branch에서만 작업하셔야 합니다.
3. 본인의 작업물이 어느정도 성과가 있으면 회의 때나 카톡방에서 설명하시고 pull하시면 됩니다.(main에 덮어쓰기)

# 해결한 내용들
- 현재 디렉터리에 파일 저장/수정 권한이 없어서 저장이나 새로운 파일 생성이 안되는 경우.
    - sudo chown -R username:username ~/your/directory/name
    - sudo chmod 777 /home/YourLinuxUsername
    - -R 옵션은 해당 디렉터리와 하위 파일에 대한 권한을 모두 수정한다.
- sudo 그룹에 현재 사용자 추가
    - sudo usermod -aG sudo your_username


# 정보
- 파일 실행 터미널에서 다음 줄 실행
- roslaunch assignment_1 parking.launch

# 5월 22일까지 해결해야 할 내용들 
1. 과제1, 2 환경 구축하기 (실행확인) 
2. git 환경 구축, 사용법 공부하기 
    - git add, commit, push 같은 기본 명령어 
    - branch나 merge같은 개념 
    - 위 내용은 유튜브에 아주 잘 나와 있으니 10~20분짜리 영상 한두개면 다 알 수 있어요. 
    - 각자 branch만들어 github에 올려보기. 
    - (중요) branch 이름은  <dev_(본인 github닉네임)> 로 만들기 
3. 과제1, 과제2 풀어보고 먼저 풀어 볼 것 정하기 
    - 2명씩 한 과제씩. 
    - 과제1팀이 끝나면 과제2에 같이 붙어서 풀 계획입니다. 
    - 토요일(5/18)까지 꼭! 자신이 풀고싶은 과제를 먼저 선택해주세요. 

- 한 줄 정리 - 환경구축, git개념 공부, 과제정하기, 하다가 안되는 것 바로바로 톡방에 물어보세요!!
