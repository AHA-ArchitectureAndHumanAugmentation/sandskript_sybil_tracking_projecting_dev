---
description: Concisely explain a technical question in simple terms
argument-hint: [question or topic]
allowed-tools: Read, Grep, Glob
---

Explain `$ARGUMENTS` in simple terms. If `$ARGUMENTS` is empty, explain whatever
was last being discussed in the conversation.

This is an EXPLANATION, not a task. Do not edit files, run commands, or start
implementing anything — just answer. If the question is about this repo, you may
read code to ground the answer, but keep the reading minimal (a couple of files
at most) and never dump source into the reply.

How to answer:

- **Be short.** Aim for under 150 words. A single tight paragraph beats a wall of
  prose. Stop when the point has landed.
- **Plain language first.** Lead with the idea in everyday words, then name the
  jargon term once so the user can look it up. Never open with jargon.
- **Use key points** (a short bullet list) when the answer has 2–5 distinct
  pieces. Use a paragraph when it is really one idea.
- **Use an analogy** when the concept is abstract — one, briefly, not a laboured
  metaphor.
- **Emojis are optional seasoning.** Use one or two only where they genuinely
  speed comprehension (⚠️ a gotcha, ✅/❌ a do-vs-don't, ➡️ a flow). Skip them
  entirely if they would just be decoration.
- **Concrete over abstract.** A tiny example or real number is worth a paragraph
  of theory.
- **Say what it's for.** Close with why it matters or when the user would care —
  that is usually the part they actually wanted.

Do not pad with caveats, disclaimers, or "it depends" hedging. If there is a real
caveat, give it one clause. If the honest answer is "you don't need to know this
unless X", say that.
