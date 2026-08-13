#!/usr/bin/env python
"""PreToolUse hook for the Bash and PowerShell tools.

Blocks obviously destructive commands before they run. This is a SPEED BUMP,
not a security boundary: it matches command strings with regexes, which are
bypassable and can false-positive. On a match it writes the reason to stderr
and exits 2; Claude Code then blocks the call and shows the message, so the
command can be re-run manually if it was actually intended.

Covers both PowerShell (the primary shell on this machine) and Unix / Git-Bash
syntax, and protects this project's runtime state that lives outside git:
the root settings.json (robot IP + projector corners), paths/ (saved
toolpaths) and surfaces/ (uploaded meshes) -- all gitignored, so a stray
recursive delete is unrecoverable.
"""
import sys
import json
import re

# (regex, reason) -- matched case-insensitively against the raw command.
_RULES = [
    # Force pushes -- can overwrite already-published history. `--force-with-lease`
    # is deliberately ALLOWED (it refuses to clobber remote commits you haven't
    # fetched, and is what the /pushgit command uses); only the unguarded
    # `--force` / `-f` forms are blocked.
    (re.compile(
        r"git\s+push\b[^|;&\n]*?"
        r"(--force\b(?!-with-lease)|(?<![\w-])-f(?![\w-]))", re.I),
     "force push (--force/-f) -- can overwrite already-pushed history; "
     "use --force-with-lease instead (dangerous on main/master)"),

    # Unix recursive remove: rm -rf, rm -r, rm -fr, rm --recursive
    # (also catches PowerShell's `rm -Recurse` alias).
    (re.compile(r"\brm\b[^|;&\n]*?(-{1,2}[a-z]*r[a-z]*\b|--recursive\b)", re.I),
     "recursive 'rm' -- deletes an entire directory tree"),

    # PowerShell recursive delete: Remove-Item / ri / rd / rmdir ... -Recurse
    (re.compile(r"\b(remove-item|ri|rd|rmdir)\b[^|;&\n]*?-recurse", re.I),
     "recursive PowerShell delete (-Recurse)"),

    # cmd recursive delete switches: rmdir /s, rd /s, del /s
    (re.compile(r"\b(rmdir|rd|del|erase)\b[^|;&\n]*?/s\b", re.I),
     "recursive cmd delete (/s)"),

    # git clean -d / -x -- would wipe the gitignored paths/, surfaces/,
    # settings.json and .venv/ in this repo.
    (re.compile(r"git\s+clean\b[^|;&\n]*?-[a-z]*[dx]", re.I),
     "git clean -d/-x -- deletes gitignored paths/, surfaces/, settings.json, .venv/"),
]

# A destructive verb combined with one of the project's protected paths,
# anywhere in the command. Catches non-recursive deletes, moves, and `>`
# overwrites that the rules above miss (e.g. `rm settings.json`, `> settings.json`).
_DESTRUCTIVE = re.compile(
    r"(\brm\b|\bremove-item\b|\bri\b|\bdel\b|\berase\b|\brmdir\b|\brd\b|"
    r"\bmove\b|\bmv\b|\bclear-content\b|\bclc\b|>\s*\S)", re.I)
_PROTECTED = re.compile(
    r"(settings\.json"
    r"|(?:^|[\s'\"/\\])paths(?:[/\\]|\s|$)"
    r"|(?:^|[\s'\"/\\])surfaces(?:[/\\]|\s|$))", re.I)


def main() -> None:
    try:
        payload = json.load(sys.stdin)
    except Exception:
        sys.exit(0)  # can't parse -> don't block

    if payload.get("tool_name") not in ("Bash", "PowerShell"):
        sys.exit(0)

    cmd = (payload.get("tool_input") or {}).get("command", "") or ""
    if not cmd.strip():
        sys.exit(0)

    hits = [reason for rx, reason in _RULES if rx.search(cmd)]
    if _DESTRUCTIVE.search(cmd) and _PROTECTED.search(cmd):
        hits.append(
            "delete/move/overwrite of a protected path "
            "(settings.json / paths/ / surfaces/)")

    if hits:
        lines = ["BLOCKED by .claude/hooks/bash_guard.py "
                 "(speed bump, not a security boundary):"]
        lines += [f"  - {h}" for h in hits]
        lines.append("If this was intentional, run it yourself or relax the rule "
                     "in .claude/hooks/bash_guard.py.")
        sys.stderr.write("\n".join(lines) + "\n")
        sys.exit(2)

    sys.exit(0)


if __name__ == "__main__":
    main()
