README FILE
# 해결한 내용들
- 파일 저장/수정 권한이 없어서 저장이나 새로운 파일 생성이 안되는 경우.
    - sudo chown -R username:username ~/your/directory/name
    - -R 옵션은 해당 디렉터리와 하위 파일에 대한 권한을 모두 수정한다.
- sudo 그룹에 현재 사용자 추가
    - sudo usermod -aG sudo your_username


# 정보
- 파일 실행 터미널에서 다음 줄 실행
- roslaunch assignment_1 parking. launch