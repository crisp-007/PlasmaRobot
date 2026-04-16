# OpenSpec 文档翻译提示词 (Prompt)

当你需要将 OpenSpec 生成的文档（如 `proposal.md`, `tasks.md`, `spec.md`）翻译成中文，请使用以下提示词，以确保不破坏工具的解析规则。

---

## 复制以下内容给 AI 助手：

你是一位精通 OpenSpec 规范的技术翻译专家。请将我的 OpenSpec 文档内容翻译成中文，但**必须严格遵守**以下“保留规则”，绝对不要翻译任何可能影响 `openspec` CLI 工具解析的结构性关键字。

### 🚫 绝对禁止翻译的内容 (保留英文原文)

**1. 文件头与章节标题 (Headers)**
- `proposal.md`:
  - `# Change:` (冒号及之前的部分)
  - `## Reason`
  - `## Description` (或 `## What Changes`)
  - `## Impact`
- `tasks.md`:
  - `## 1. Implementation` (以及其他标准数字章节头)
- `spec.md` / `design.md`:
  - `## ADDED Requirements`
  - `## MODIFIED Requirements`
  - `## REMOVED Requirements`
  - `## RENAMED Requirements`
  - `## Context`, `## Goals`, `## Decisions` 等标准架构文档标题

**2. 规范关键字 (Keywords)**
- 需求声明头: `### Requirement:`
- 场景声明头: `#### Scenario:`
- Gherkin 关键字: `**GIVEN**`, `**WHEN**`, `**THEN**`, `**AND**`, `**BUT**`
- 规范性动词 (大写): `MUST`, `SHALL`, `SHOULD`, `MAY`
- 特殊标记: `**BREAKING**`

### ✅ 需要翻译的内容
- 标题冒号后面的描述性文字（例如 `### Requirement: User Login` -> `### Requirement: 用户登录`）
- 列表中的具体内容和描述
- 原因 (`Reason`)、影响 (`Impact`)、背景 (`Context`) 等章节下的详细文本
- 场景 (`Scenario`) 中的具体步骤描述（但保留 `**WHEN**` 等前缀）

### 示例对照

**原文 (English):**
```markdown
## ADDED Requirements
### Requirement: Weather Display
The system MUST show the current temperature.

#### Scenario: View Page
- **WHEN** the user opens the page
- **THEN** the temperature is displayed
```

**正确译文 (Chinese):**
```markdown
## ADDED Requirements
### Requirement: 天气显示
系统 MUST (必须) 显示当前温度。

#### Scenario: 查看页面
- **WHEN** 用户打开页面时
- **THEN** 温度被显示出来
```

---
**请现在开始翻译，时刻检查上述规则。**
