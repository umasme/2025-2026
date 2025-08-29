## Code Style
- **Language**: Python 3.x
- **Formatting**: Use `clang-format` with the repo's `.clang-format` file before committing.
- **Naming**:
  - Variables: `snake_case`
  - Functions: `camelCase`
  - Constants/macros: `UPPER_CASE`
  - Classes: `PascalCase`
- Avoid "magic numbers" — define

---

## Git Workflow
- **Default branch**: `main` (stable releases only)
- **Development branch**: `develop` (integration of tested features)
- **Feature branches**: `feature/<short-description>`
- **Commit messages**: Follow [Conventional Commits](https://www.conventionalcommits.org/)  
  Examples:  
  - `feat: add LPS22HB driver`  
  - `fix: correct SPI CS pin for SD card`

---

## Documentation Expectations
- All new modules → add a **minimal example** in `/examples`.

---

## Testing Before Merge
- Pass all hardware bring-up tests relevant to the modified code.
- For I²C devices: verify with the scanner example before integration.
- For SPI devices: verify standalone before shared-bus testing.
- Run integrated telemetry + logging for ≥60s without errors before merging to `develop`.

---

## Pull Request Checklist
- [ ] Code builds without warnings on target board.
- [ ] Examples updated or new example added.
- [ ] Relevant docs updated.
- [ ] Test plan passed and results noted in PR description.
