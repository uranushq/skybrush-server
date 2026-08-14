<!-- ENTRIES -->

## 2026-08-14 18:08:37 +0900 — `30936ebe` ..

_branch: dev · author: directorBae <bjw020615@gmail.com>_

**요약**: JR 보드의 reboot/redownload 제어를 온보드 HTTP API에서 UDP 명령·ack 방식으로 전환해, 보드가 HTTP 서버를 완전히 없앤 구조를 마무리했다.

**주요 변경점**:
- `commands.py` 신설: `reboot`/`redownload`를 헬스 UDP 포트(기본 16550)로 JSON 데이터그램 전송 후 보드 ack를 대기(`to_thread`로 블로킹 소켓을 이벤트 루프 밖에서 실행, 기본 timeout 3초).
- 기존 HTTP 프록시 `health.py` 삭제, `health_udp.py`의 `JRBoardError` import를 `commands`로 이관.
- `extension.py`에서 reboot/redownload 엔드포인트가 UDP 명령을 쓰도록 변경하고, 모듈 전역 `health_port`(설정에서 주입)를 명령 전송에 재사용(`overridden`으로 전달). 문서화 주석도 "보드에 HTTP 서버 없음"으로 갱신.
- `DEV_LOG.md`에 직전 커밋(`f1eca657`) 분석 항목 추가.

**의미/영향**: 앞선 커밋에서 헬스 조회를 UDP 푸시로 바꾼 데 이어, 이번엔 제어 명령까지 UDP로 통일해 보드의 HTTP 스택 의존을 완전히 제거했다. 이로써 상태 수신·제어가 단일 포트/프로토콜로 일원화되어 "no telem" 병목을 유발하던 HTTP 폴링 구조가 정리되고, 펌웨어 측 부담과 코드 경로가 단순해졌다.

**주의/리스크**: UDP는 비신뢰 전송이라 명령/ack 유실 시 timeout으로 "unreachable"처럼 실패할 수 있고, 서버의 `health_port`가 펌웨어 `CFG_HEALTH_UDP_PORT`와 정확히 일치해야 한다. 또한 보드가 쇼 PLAYING 중이면 명령을 무시해 timeout이 정상 동작과 실패가 구분되지 않으며, 인증 없는 UDP 명령이므로 동일 네트워크 내 오·악용 여지에 대한 검증이 필요하다.

---


## 2026-08-14 18:01:26 +0900 — `f1eca657` 0814

_branch: dev · author: directorBae <bjw020615@gmail.com>_

**요약**: JR 보드 상태 조회를 HTTP 폴링에서 UDP 푸시 수신 방식으로 전환하고, 경로 계획기에 착륙 시 넓은 간격으로 펼쳐 내리는 옵션을 추가했다.

**주요 변경점**:
- `jr_control`에 `health_udp.py` 신설: 보드가 UDP(기본 16550 포트)로 밀어주는 상태를 캐시하고, `/health/<ip>`는 최근 수신값을 반환(15초 초과 시 stale 처리).
- 확장에서 기존 동기 HTTP `get_health` 폴링 제거, `run()`이 `sleep_forever` 대신 UDP 리스너를 상시 실행. `health_host`/`health_port` 설정 스키마 추가.
- `health.py`에서 `/health` 프록시 삭제(reboot·redownload만 유지).
- `path_planner`에 `landing_targets` 도입 및 `DEFAULT_LANDING_SPACING=4.0`: 착륙 복귀 구간을 원래 이륙 지점 대신 이륙 스테이징과 동일한 솔버로 넓게 펼쳐 착지.

**의미/영향**: 약 30대 보드를 HTTP로 동기 폴링하던 구조가 "no telem" 병목·연결 끊김의 원인이었는데, 이를 보드 주도 UDP 푸시로 바꿔 텔레메트리 지연과 부하를 줄였다. 착륙 간격 옵션은 좁은 이륙 배치에서도 안전한 착지 분산을 가능하게 해 실제 군집 운용 안정성을 높인다.

**주의/리스크**: UDP는 비신뢰 전송이라 패킷 유실 시 stale 판정으로 "unreachable"이 될 수 있고, 서버 리스너 포트가 보드 펌웨어의 `CFG_HEALTH_UDP_PORT`와 일치해야 한다. 또한 캐시가 프로세스 메모리에 저장돼 재시작 시 보드가 다시 보고할 때까지 상태가 비며, 브로드캐스트/멀티 보드 환경에서 인증 없는 UDP 수신의 신뢰성 검증이 필요하다.

---


## 2026-07-03 17:08:21 +0900 — `3e63e3ad` claude commit 분석 완료

_branch: dev · author: directorBae <bjw020615@gmail.com>_

**요약**: 모든 git 커밋 후 자동으로 커밋을 이메일로 보고하고, 선택적으로 Claude 헤드리스 분석을 첨부하는 post-commit 훅 스크립트를 신규 도입했다.

**주요 변경점**:
- `.claude/hooks/git-commit-report.py`(287줄) 신규 추가 — git-hook 모드/Claude Code 훅 모드 이중 동작, 커밋 메타데이터·diffstat 수집, `claude -p` 헤드리스 호출로 커밋 분석 생성.
- 분석 결과를 `docs/DEV_LOG.md`의 `<!-- ENTRIES -->` 마커 아래에 누적 기록하고, SMTP(STARTTLS 587/SSL 465)로 이메일 발송.
- 설정은 환경변수 → `.claude/settings.local.json`의 `env` 블록 순으로 로드(SMTP 접속정보, `COMMIT_ANALYZE`, `COMMIT_ANALYZE_MODEL`=기본 opus 등).
- `.gitignore`에 `.claude/settings.local.json` 추가 — SMTP 비밀번호 등 시크릿 커밋 방지.

**의미/영향**: 개발 워크플로에 자동화 계층을 얹은 변경으로, 제품 기능 자체가 아니라 커밋마다 변경 이력을 이메일·개발 로그로 남겨 추적성을 높이는 인프라다. 실제 앱 코드는 건드리지 않고 커밋 성공 후에만 동작하며, 자격증명 누락·분석 실패·메일 오류 시에도 항상 exit 0으로 커밋을 방해하지 않도록 설계되어 안전하게 도입 가능하다.

**주의/리스크**: 시크릿은 gitignore로 보호되지만 `settings.local.json` 최초 미설정 시 이메일이 조용히 스킵되므로 각 개발자 환경에서 SMTP 설정이 필요하다. 또한 커밋마다 `claude -p`(opus)를 최대 240초까지 호출하므로 커밋 후 지연·API 비용이 발생할 수 있고, diff/문서를 외부(Claude·SMTP)로 전송하는 만큼 민감 코드 유출 관점의 검토가 필요하다.

---

