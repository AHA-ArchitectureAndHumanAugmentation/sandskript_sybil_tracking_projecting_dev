---
description: Fetch and fast-forward the current branch from origin (or from a given GitHub URL)
argument-hint: [github url]
allowed-tools: Bash(git status:*), Bash(git branch:*), Bash(git rev-parse:*), Bash(git remote:*), Bash(git fetch:*), Bash(git merge:*), Bash(git log:*), Bash(git diff:*), Bash(git stash:*)
---

Bring the local repo up to date with GitHub. `$ARGUMENTS` selects the source:

- **empty** → pull from `origin` (the normal case).
- **a GitHub URL** → pull from that URL instead. If the URL matches a remote
  that already exists (`git remote -v`), use that remote's name. Otherwise fetch
  the URL directly — do NOT permanently add a new remote unless the user asks.

Follow these steps in order and STOP if a step fails.

1. Determine the current branch: `git branch --show-current`. If it is empty
   (detached HEAD), stop and tell the user — there is no branch to update.

2. Check the working tree: `git status --porcelain`.
   - If it is dirty, STOP and show the user what is uncommitted, then ask
     whether to stash it (`git stash push -u`), commit it first, or abort.
     Never discard or overwrite local edits without an explicit answer.
   - Note: on this repo the machine-specific ENVPY paths in `run.bat`,
     `run_replay.bat`, `run_stitch.bat`, `.mcp.json` and `CLAUDE.md` are a
     recurring source of local edits (see CLAUDE.md "Run / test").

3. Fetch, without touching the working tree yet:
   - No argument: `git fetch origin`
   - Named remote: `git fetch <remote>`
   - Bare URL: `git fetch <url> <branch>` (the result is `FETCH_HEAD`).

4. Show what is incoming BEFORE applying it:
   `git log --oneline HEAD..<upstream>` and `git diff --stat HEAD..<upstream>`,
   where `<upstream>` is `origin/<branch>`, `<remote>/<branch>` or `FETCH_HEAD`.
   - If there is nothing incoming, report "already up to date" and stop.
   - Summarize the incoming commits in one or two lines.

5. Apply it: `git merge --ff-only <upstream>`.
   - `--ff-only` refuses to create a merge commit, so it succeeds only when the
     local branch has no commits of its own. If it fails, the branches have
     diverged — STOP, report the divergence, and let the user choose between
     rebasing, merging, or resetting. Do NOT pick one for them.

6. Report what landed: old → new commit, the number of commits and files, and a
   short summary of what actually changed (features, not just filenames).

7. Check whether the update needs follow-up work, and say so explicitly:
   - Did `requirements.txt`, `requirements-dev.txt` or `environment.yml` change?
     If so, the `sandskript` env needs updating — offer to install the new
     dependencies (`<ENVPY> -m pip install ...` / `conda install -n sandskript
     -c conda-forge ...`; never bare `pip`).
   - Did the ENVPY paths come from a different machine? If so, offer to point
     them back at this machine.
   - If Python files changed, remind the user a running app needs a restart.
   - If a stash was made in step 2, restore it (`git stash pop`) and report any
     conflicts.

8. Offer to run the unit tests (`<ENVPY> -m pytest -q -m "not integration"`) so
   the pulled state is verified — but only run them if the user agrees or the
   pull touched Python code.

Note: this command only moves the local branch forward. It never pushes, never
force-updates, and never rewrites local commits.
