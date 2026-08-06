# AI DEVELOPMENT ENTRY

在分析或修改本仓库前，必须先阅读并遵守：

1. `docs/DEVELOPMENT_REQUIREMENTS.md`
2. `docs/ARCHITECTURE_BOUNDARIES.md`
3. `docs/CODE_MAP.md`
4. `docs/LOGGING_GUIDE.md`
5. `docs/NAVIGATION_WORKFLOW.md`

其中 `docs/DEVELOPMENT_REQUIREMENTS.md` 是本项目面向 AI 的总开发约束。若需求与现有架构冲突，先说明冲突和影响，不得通过复制状态机、跨层调用或临时旁路完成修改。新增相似逻辑前必须先查找现有实现；确认旧实现错误时应在验证调用关系后替换并清理旧代码，不得让新旧逻辑长期并存。新增代码必须简洁、高效、无重复职责。
