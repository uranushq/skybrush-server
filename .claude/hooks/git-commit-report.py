#!/usr/bin/env python3
"""Email a report after every git commit, optionally with a Claude analysis.

Runs in two modes:

* git-hook mode (pass --git-hook): invoked by .git/hooks/post-commit, so it fires
  on EVERY commit in this repo -- terminal, IDE, or Claude Code. git only runs
  post-commit after a commit actually succeeds, so there are no false reports.

* Claude Code hook mode (no arg): reads hook-input JSON on stdin. Kept for
  completeness; the git hook already covers Claude Code commits, so this repo wires
  up only the git hook to avoid duplicate emails.

The script gathers the latest commit's metadata and diffstat, optionally asks Claude
(headless `claude -p`) to analyze the commit, appends that analysis to
docs/DEV_LOG.md, and emails the whole thing via SMTP.

SMTP settings are read from environment variables first, then fall back to the "env"
block of .claude/settings.local.json (gitignored, so secrets are never committed):

  SMTP_HOST    e.g. smtp.gmail.com (Google Workspace) or smtp.office365.com (M365)
  SMTP_PORT    587 for STARTTLS, 465 for implicit SSL   (default: 587)
  SMTP_USER    sending account, e.g. you@uranushq.com
  SMTP_PASS    app password -- NOT your normal login password
  REPORT_TO    recipient address, e.g. bjw020615@uranushq.com
  REPORT_FROM  optional; defaults to SMTP_USER

Analysis settings (same source: env or settings.local.json "env"):

  COMMIT_ANALYZE        "0"/"false"/"no"/"off" to disable   (default: on)
  COMMIT_ANALYZE_MODEL  model for the analysis               (default: opus)
  COMMIT_CLAUDE_BIN     explicit path to the claude binary   (default: auto-detect)

The commit is never affected: on missing credentials, a failed analysis, or SMTP
errors the script just prints a note and exits 0.
"""

import json
import os
import smtplib
import ssl
import subprocess
import sys
from email.message import EmailMessage


GIT_HOOK_MODE = "--git-hook" in sys.argv


def emit(message):
    """One-line status. Plain text under git; systemMessage JSON under Claude Code.

    Never raises: on a console codec that can't encode the text (e.g. cp949 on
    Windows) it falls back to a utf-8 byte write instead of crashing the hook."""
    text = message if GIT_HOOK_MODE else json.dumps({"systemMessage": message})
    try:
        print(text)
    except UnicodeEncodeError:
        try:
            sys.stdout.buffer.write((text + "\n").encode("utf-8", "replace"))
        except Exception:
            pass


def run_git(repo, *args):
    result = subprocess.run(
        ["git", "-C", repo, *args],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return result.stdout.strip()


def load_local_settings_env(repo):
    """Read the "env" block from .claude/settings.local.json, if present."""
    path = os.path.join(repo, ".claude", "settings.local.json")
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle).get("env", {}) or {}
    except Exception:
        return {}


def find_claude(override=""):
    """Locate the claude executable (override -> PATH -> known locations). '' if none."""
    from shutil import which

    if override and os.path.exists(override):
        return override
    for name in ("claude", "claude.exe"):
        found = which(name)
        if found:
            return found
    home = os.path.expanduser("~")
    for path in (
        os.path.join(home, ".local", "bin", "claude.exe"),
        os.path.join(home, ".local", "bin", "claude"),
    ):
        if os.path.exists(path):
            return path
    return ""


def read_capped(path, limit):
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            text = handle.read()
    except Exception:
        return ""
    return text if len(text) <= limit else text[:limit] + "\n...(생략)..."


def analyze_commit(repo, meta, model, claude_bin):
    """claude 헤드리스로 커밋 diff를 분석해 마크다운 텍스트를 반환. 실패 시 ''."""
    if not claude_bin:
        return ""
    status_doc = read_capped(os.path.join(repo, "docs", "PROJECT_STATUS.md"), 6000)
    diff = run_git(repo, "show", "HEAD", "--format=", "-p")
    if len(diff) > 12000:
        diff = diff[:12000] + "\n...(diff 생략)..."

    prompt = (
        "너는 이 소프트웨어 프로젝트의 커밋을 분석하는 어시스턴트다.\n"
        "아래 [프로젝트 개요]를 참고해, [이번 커밋]이 무엇을 바꿨고 프로젝트 진행에\n"
        "어떤 의미인지 한국어로 간결하게 분석하라. 도구를 쓰지 말고 주어진 정보만으로\n"
        "판단하라. 반드시 아래 마크다운 형식으로만 출력하라(다른 제목/서론 없이):\n\n"
        "**요약**: (한 줄)\n"
        "**주요 변경점**:\n- ...\n"
        "**의미/영향**: (기존 기능·진행상황 관점, 2-3문장)\n"
        "**주의/리스크**: (없으면 \"특이사항 없음\")\n\n"
        f"[프로젝트 개요]\n{status_doc}\n\n"
        f"[이번 커밋]\n해시: {meta['hash']}\n브랜치: {meta['branch']}\n"
        f"메시지: {meta['subject']}\n{meta['body']}\n\n"
        f"변경 통계:\n{meta['stat']}\n\ndiff:\n{diff}\n"
    )
    try:
        out = subprocess.run(
            [claude_bin, "-p", "--model", model],
            input=prompt,
            cwd=repo,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=240,
        )
        return out.stdout.strip()
    except Exception:
        return ""


def append_dev_log(repo, meta, analysis):
    """DEV_LOG.md의 <!-- ENTRIES --> 마커 바로 아래에 최신 항목을 추가."""
    log = os.path.join(repo, "docs", "DEV_LOG.md")
    marker = "<!-- ENTRIES -->"
    entry = (
        f"\n## {meta['date']} — `{meta['hash']}` {meta['subject']}\n\n"
        f"_branch: {meta['branch']} · author: {meta['author']}_\n\n"
        f"{analysis}\n\n---\n"
    )
    try:
        if os.path.exists(log):
            with open(log, encoding="utf-8", errors="replace") as handle:
                content = handle.read()
        else:
            content = marker + "\n"
        if marker in content:
            content = content.replace(marker, marker + "\n" + entry, 1)
        else:
            content += "\n" + entry
        os.makedirs(os.path.dirname(log), exist_ok=True)
        with open(log, "w", encoding="utf-8") as handle:
            handle.write(content)
    except Exception:
        pass


def main():
    if GIT_HOOK_MODE:
        # git runs post-commit from the worktree root.
        repo = run_git(os.getcwd(), "rev-parse", "--show-toplevel") or os.getcwd()
    else:
        try:
            payload = json.load(sys.stdin)
        except Exception:
            payload = {}
        command = (payload.get("tool_input") or {}).get("command", "")
        if "git commit" not in command:
            sys.exit(0)
        repo = payload.get("cwd") or os.getcwd()

    commit_hash = run_git(repo, "rev-parse", "--short", "HEAD")
    if not commit_hash:
        sys.exit(0)

    branch = run_git(repo, "rev-parse", "--abbrev-ref", "HEAD")
    subject = run_git(repo, "log", "-1", "--format=%s")
    author = run_git(repo, "log", "-1", "--format=%an <%ae>")
    date = run_git(repo, "log", "-1", "--format=%cd", "--date=iso")
    body = run_git(repo, "log", "-1", "--format=%b")
    stat = run_git(repo, "show", "--stat", "--format=", "HEAD")

    local_env = load_local_settings_env(repo)

    def setting(name, default=None):
        value = os.environ.get(name)
        if value is None:
            value = local_env.get(name)
        return value if value is not None else default

    # Claude 커밋 분석 -- 실패해도 메일/커밋에 영향 없음
    meta = {
        "hash": commit_hash,
        "branch": branch,
        "subject": subject,
        "author": author,
        "date": date,
        "body": body,
        "stat": stat,
    }
    analysis = ""
    if setting("COMMIT_ANALYZE", "1").lower() not in ("0", "false", "no", "off"):
        model = setting("COMMIT_ANALYZE_MODEL", "opus")
        claude_bin = find_claude(setting("COMMIT_CLAUDE_BIN", ""))
        emit(f"commit-report: {model}로 커밋 {commit_hash} 분석 중...")
        analysis = analyze_commit(repo, meta, model, claude_bin)
        if analysis:
            append_dev_log(repo, meta, analysis)

    report = (
        f"New commit in {repo}\n\n"
        f"Commit : {commit_hash}\n"
        f"Branch : {branch}\n"
        f"Author : {author}\n"
        f"Date   : {date}\n"
        f"Subject: {subject}\n"
    )
    if body:
        report += f"\n{body}\n"
    report += f"\nChanges:\n{stat or '(no file changes)'}\n"
    if analysis:
        report += "\n" + ("=" * 52) + "\n[Claude 분석]\n" + ("=" * 52) + "\n" + analysis + "\n"

    host = setting("SMTP_HOST")
    port = int(setting("SMTP_PORT", "587"))
    user = setting("SMTP_USER")
    password = setting("SMTP_PASS")
    to_addr = setting("REPORT_TO")
    from_addr = setting("REPORT_FROM") or user

    if not all([host, user, password, to_addr]):
        emit(
            "commit-report: SMTP settings incomplete -- email skipped. "
            "Fill SMTP_HOST/SMTP_USER/SMTP_PASS/REPORT_TO in "
            ".claude/settings.local.json"
        )
        sys.exit(0)

    msg = EmailMessage()
    tag = "[commit+분석]" if analysis else "[commit]"
    msg["Subject"] = f"{tag} {branch} {commit_hash} - {subject}"
    msg["From"] = from_addr
    msg["To"] = to_addr
    msg.set_content(report)

    try:
        context = ssl.create_default_context()
        if port == 465:
            with smtplib.SMTP_SSL(host, port, context=context, timeout=20) as server:
                server.login(user, password)
                server.send_message(msg)
        else:
            with smtplib.SMTP(host, port, timeout=20) as server:
                server.starttls(context=context)
                server.login(user, password)
                server.send_message(msg)
        emit(f"commit-report: emailed {commit_hash} to {to_addr}")
    except Exception as exc:  # never disrupt the commit on a mail failure
        emit(f"commit-report: email failed ({exc})")

    sys.exit(0)


if __name__ == "__main__":
    main()
