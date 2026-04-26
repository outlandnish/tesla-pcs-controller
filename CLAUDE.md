- Think before acting. Read existing files before writing code.
- Be concise in output but thorough in reasoning.
- Prefer editing over rewriting whole files.
- Do not re-read files you have already read.
- Test your code before declaring done.
- No sycophantic openers or closing fluff.
- Keep solutions simple and direct.

- This project aims to communicate with a Tesla Model 3 PCS by emulating the rest of the car
- Based on older OpenInverter firmware provided in the old folder
- raw data traces are data/traces from a production Tesla Model 3
- Currently running into two alerts while enabling the PCS (PCS_a024_vcFrontMia, PCS_a107_vcPcsDCDCInterfaceMia)

- Use DEBUG_SERIAL instead of Serial for printing any debug statements
