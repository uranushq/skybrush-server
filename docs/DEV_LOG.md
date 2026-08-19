<!-- ENTRIES -->

## 2026-08-19 21:49:43 +0900 — `3eabd063` ss

_branch: dev · author: directorBae <bjw020615@gmail.com>_

**요약**: 직전 devlog 커밋(`12a0d0a8`)에 대한 분석 항목을 `docs/DEV_LOG.md`에 추가한 문서 전용 커밋이다.

**주요 변경점**:
- `docs/DEV_LOG.md` 상단(`<!-- ENTRIES -->` 아래)에 `12a0d0a8` "devlog" 항목 18줄 추가.
- 추가된 내용은 `238e25be`("led udp") 분석을 DEV_LOG에 기록한 커밋 자체를 다시 요약한 것으로, "HTTP 제거→UDP 통일" 및 LED UDP 확장 흐름을 서술.
- 코드·기능 변경 없이 문서만 수정(18줄 추가).

**의미/영향**: 소스 동작에는 영향이 없고, 커밋 단위로 분석 로그를 누적하는 DEV_LOG 관행이 계속 유지되고 있음을 보여준다. 다만 이번 항목은 앞선 devlog 커밋(문서 기록 커밋)에 대한 기록이라 "문서를 기록한 커밋을 다시 문서화"하는 메타 계층이 한 단계 생긴 형태다.

**주의/리스크**: 커밋 메시지가 `ss`로 내용을 전혀 설명하지 못하며, devlog을 문서화하는 devlog가 반복되면 로그가 실제 코드 변경보다 문서 갱신 이력으로 채워질 수 있으니 기록 대상 기준을 정리해 둘 필요가 있다.

---


## 2026-08-16 18:45:37 +0900 — `12a0d0a8` devlog

_branch: dev · author: directorBae <bjw020615@gmail.com>_

**요약**: 직전 커밋(`238e25be` "led udp")의 분석 항목을 `docs/DEV_LOG.md`에 추가한 개발 로그 갱신 커밋이다.

**주요 변경점**:
- `docs/DEV_LOG.md` 상단(`<!-- ENTRIES -->` 아래)에 `238e25be` 항목 19줄 추가.
- 추가된 내용은 UDP `led` 명령 도입(평문 전송 경로 `send_raw_command`, `POST /led/<ip>` 엔드포인트, 색상 4채널 clamp) 및 HTTP 제거→UDP 전환 아키텍처의 진행을 서술.
- 코드/기능 변경 없이 문서만 수정.

**의미/영향**: 실제 소스에는 영향이 없고, "HTTP 제거→UDP 통일" 흐름의 LED 확장 작업(`238e25be`)을 사후 기록해 개발 히스토리 추적성을 유지하는 커밋이다. 최신순으로 항목이 쌓이는 DEV_LOG 관행이 유지되고 있어, 커밋 단위 문서화 프로세스가 정착 단계임을 보여준다.

**주의/리스크**: 특이사항 없음

---


## 2026-08-16 18:42:49 +0900 — `238e25be` led udp

_branch: dev · author: directorBae <bjw020615@gmail.com>_

**요약**: JR 보드 제어에 UDP `led` 명령을 추가해, HTTP 제거 후 UDP 전환 흐름에 LED 점등/소등(배선 점검용) 기능과 `POST /led/<ip>` 엔드포인트를 신설했다.

**주요 변경점**:
- `commands.py`에 `format_led_body`·`post_led`·`send_raw_command`를 추가하고, `_send_and_wait`를 `cmd` 문자열 대신 `body: bytes` + `label` 방식으로 리팩터링(JSON 래퍼 없이 평문 전송 가능).
- 펌웨어가 본문을 substring 매칭(`redownload`→`reboot`→`led` 순)하고 `sscanf`로 색상 4채널을 파싱한다는 점에 맞춰, LED 본문은 JSON이 아닌 평문(`led R G B W` 또는 `led off`)으로 생성하고 각 채널을 0~255로 clamp.
- `extension.py`에 `POST /led/<ip>` 라우트 추가: JSON 바디(`red/green/blue/white/off`)를 정수 검증 후 `post_led` 호출, 실패 시 400/502 반환.
- `DEV_LOG.md`에 직전 문서 커밋(`8beb20f1`) 분석 항목 추가.

**의미/영향**: `30936ebe`에서 시작된 "HTTP 서버 제거 → UDP 명령/ack 통일" 아키텍처가 reboot·redownload에 이어 LED까지 확장되며 제어 계열이 UDP로 완성 단계에 접어들었다. 특히 인자를 본문에서 파싱하는 명령을 위해 평문 전송 경로(`send_raw_command`)를 분리한 것은, 기존 JSON 기반 명령과 펌웨어 파싱 규칙 사이의 충돌을 구조적으로 해결한 실질적 진전이다.

**주의/리스크**: 펌웨어가 본문을 순서 있는 substring 매칭으로 처리하므로 `led` 본문에 "reboot"·"redownload" 같은 상위 키워드가 섞이면 오작동할 수 있어(코드 주석에도 명시), 향후 본문 포맷 변경 시 이 규칙을 반드시 지켜야 한다. 또한 LED는 소등/다음 쇼 재생 전까지 계속 켜져 있고 `PLAYING` 중에는 무시되어 타임아웃되는 동작 특성에 유의해야 한다.

---


## 2026-08-14 18:19:23 +0900 — `8beb20f1` .

_branch: dev · author: directorBae <bjw020615@gmail.com>_

**요약**: 직전 커밋(`4c6c4ba7`)의 분석 항목을 `docs/DEV_LOG.md`에 추가한 문서 전용 커밋으로, 로그가 로그를 기록하는 자기참조 체인을 또 한 단계 연장했다.

**주요 변경점**:
- `docs/DEV_LOG.md` 최상단(`<!-- ENTRIES -->` 바로 아래)에 `4c6c4ba7` 커밋 분석 항목 18줄 추가.
- 추가 항목의 내용은 "`4c6c4ba7`가 그 직전 커밋(`78975e95`)을 로그에 기록한 문서 커밋"이라는 요약(즉, 문서 커밋을 기록한 문서 커밋을 다시 기록).
- 코드·로직 변경 없이 로그 문서만 갱신(1 file changed, +18).

**의미/영향**: 실제 아키텍처 작업(`30936ebe`의 HTTP 제거·UDP 명령/ack 전환) 기록이 `30936ebe → 78975e95 → 4c6c4ba7 → 8beb20f1`로 이어지는 메타 체인으로 계속 누적되고 있다. 기능 동작에는 아무 영향이 없으며, 실질 진행 없이 로그 문서만 비대해지는 보조성 커밋이다.

**주의/리스크**: 커밋 메시지가 "."뿐이라 이력 추적성이 낮고, 문서 커밋이 서로를 참조하며 반복 생성되어 로그 잡음이 증가하고 있다. 자동 로깅을 코드 변경이 있는 커밋에만 적용하도록 범위를 제한해 메타 체인 누적을 끊는 것을 권장한다.

---


## 2026-08-14 18:16:50 +0900 — `4c6c4ba7` .

_branch: dev · author: directorBae <bjw020615@gmail.com>_

**요약**: 직전 커밋(`78975e95`)의 분석 항목을 `docs/DEV_LOG.md`에 추가한 문서 전용 커밋으로, 개발 로그 자기참조 체인을 한 단계 더 이어붙인 것이다.

**주요 변경점**:
- `docs/DEV_LOG.md` 최상단(`<!-- ENTRIES -->` 바로 아래)에 `78975e95` 커밋 분석 항목 18줄 추가.
- 추가된 항목의 내용은 "`78975e95`가 다시 그 직전 커밋(`30936ebe`, JR 보드 reboot/redownload 제어의 UDP 전환)을 로그에 기록한 문서 커밋"이라는 요약이다.
- 코드·로직 변경 없이 로그 문서만 갱신(1 file changed, +18).

**의미/영향**: 실제 아키텍처 작업(`30936ebe`의 HTTP 스택 제거 및 UDP 명령·ack 전환)에 대한 기록이 `30936ebe` → `78975e95` → `4c6c4ba7`로 이어지는 "로그를 기록한 커밋을 다시 로그에 기록"하는 메타 체인 형태로 누적되고 있다. 기능 동작에는 영향이 없으며, 변경 이력의 가시성을 높이는 보조 성격의 커밋이다.

**주의/리스크**: 커밋 메시지가 "."뿐이라 이력 추적성이 낮으므로 의미 있는 메시지 사용을 권장한다. 또한 실제 코드 변경 없는 문서 커밋이 서로를 참조하며 반복 생성되면 로그가 실질 진행 없이 비대해질 수 있어, 자동 로깅이 코드 커밋 대상만 기록하도록 범위를 제한하는 것이 바람직하다.

---


## 2026-08-14 18:15:34 +0900 — `78975e95` .

_branch: dev · author: directorBae <bjw020615@gmail.com>_

**요약**: 직전 커밋(`30936ebe`, JR 보드 reboot/redownload 제어의 UDP 전환)에 대한 분석 항목을 `docs/DEV_LOG.md`에 추가한 문서 전용 커밋이다.

**주요 변경점**:
- `docs/DEV_LOG.md` 최상단(`<!-- ENTRIES -->` 바로 아래)에 `30936ebe` 커밋 분석 항목 19줄 추가.
- 추가 내용은 `commands.py` 신설, HTTP 프록시 `health.py` 삭제, `extension.py`의 엔드포인트 UDP 전환 등 제어 명령의 UDP 일원화 작업을 요약·기록.
- 코드/로직 변경은 없으며 로그 문서만 갱신(1 file changed, +19).

**의미/영향**: 보드의 HTTP 스택 제거 및 UDP 명령·ack 전환이라는 실제 아키텍처 작업의 이력을 개발 로그에 남겨, 상태 수신·제어를 단일 포트/프로토콜로 통일한 진행 상황을 추적 가능하게 만든다. 기능 동작에는 영향이 없고, 변경 이력의 가시성·연속성을 높이는 보조 성격의 커밋이다.

**주의/리스크**: 커밋 메시지가 "."로 내용을 전혀 설명하지 못해 이력 추적성이 떨어지므로 의미 있는 메시지 사용을 권장한다. 로그 기재 내용 자체의 리스크(UDP 유실 시 timeout, `health_port`와 펌웨어 포트 일치 필요, 인증 없는 UDP 명령)는 실제 코드 커밋(`30936ebe`) 쪽에서 관리되어야 한다.

---


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

