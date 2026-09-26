# forodobots — Agent Instructions

## Project Context
- **Tech Stack:** Python
- **Infrastructure:** AWS + Kubernetes

## Multi-Agent Pipeline

This project uses a multi-agent orchestration system. Agent instructions are imported from `.gemini/agents/`:

@.gemini/agents/orchestrator.md
@.gemini/agents/debugger.md
@.gemini/agents/researcher.md
@.gemini/agents/planner.md
@.gemini/agents/dev.md
@.gemini/agents/qa.md
@.gemini/agents/reviewer.md
@.gemini/agents/reviewer-fix.md
@.gemini/agents/architecture-reviewer.md

Start by reading the **orchestrator** agent instructions when working on any ticket or feature.
