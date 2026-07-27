# CLAUDE.md

> Preferências de quem trabalha neste projeto + mapa de fases. Enxuto, sempre presente.

## Como trabalhar comigo

- <ex: antes de codar algo não-trivial, alinhe o plano e espere aprovação.>
- <ex: tarefas pequenas (ajuste de config/doc): faça direto.>
- Em dúvida sobre requisito, **pergunte** — não adivinhe.
- Leia `AGENTS.md`; abra `.ai/docs/` sob demanda.

## Contexto deste projeto

- Este é um **fork** do `ethz-asl/kalibr`. Concentre customizações em `scripts/` e preserve
  compatibilidade com o upstream (facilita rebase/merge).
- As saídas deste Kalibr serão **integradas ao AQUA-SLAM** (`../AQUA-SLAM`). Ponte de integração:
  `.ai/docs/domain/cam-imu-calibration.md`.

## Preferências de qualidade

- <TDD? Em que código? Teste primeiro?>
- Aplique YAGNI e DRY.
- Commits no formato: <ex: Conventional Commits — a confirmar>.
