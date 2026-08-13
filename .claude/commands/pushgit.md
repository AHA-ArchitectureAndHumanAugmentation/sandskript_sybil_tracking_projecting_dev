---
description: Commit all local changes and force-push (replace) the current branch on origin
argument-hint: [commit message]
allowed-tools: Bash(git status:*), Bash(git branch:*), Bash(git rev-parse:*), Bash(git remote:*), Bash(git add:*), Bash(git diff:*), Bash(git log:*), Bash(git commit:*), Bash(git push:*)
---

Push every local change to GitHub and replace (overwrite) the current branch on
`origin`. Follow these steps in order and STOP if a step fails.

1. Determine the current branch: `git branch --show-current`. If it is empty
   (detached HEAD), stop and tell the user — there is no branch to push.

2. Show what will be pushed: run `git status --short` and
   `git log origin/<branch>..HEAD --oneline` (ignore an error if the remote
   branch does not exist yet). Briefly summarize the pending changes.

3. Stage everything: `git add -A`.

4. Commit:
   - If `$ARGUMENTS` is non-empty, use it verbatim as the commit message.
   - Otherwise write a concise message summarizing the staged diff.
   - If there is nothing staged (clean tree), skip the commit and note it —
     you will still (force-)push in case local commits are ahead of origin.
   - End the commit message with:
     Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>

5. Replace the remote branch:
   `git push --force-with-lease origin <branch>`
   - `--force-with-lease` overwrites the remote branch with your local state
     but aborts if origin has commits you have not fetched. If it aborts,
     STOP and report the divergence to the user — do NOT escalate to a plain
     `--force` unless the user explicitly asks.
   - If the branch has no upstream yet, use
     `git push --force-with-lease -u origin <branch>`.

6. Report the result: the branch, the commit that is now at the tip, and
   confirmation that origin was updated.

Note: this rewrites the remote branch history. It is intentional here ("replace
the current branch"), but never run it against a shared branch others may have
based work on without confirming first.
