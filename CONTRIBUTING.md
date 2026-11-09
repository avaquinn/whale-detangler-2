## Contributing

## Commit Etiquette
Commits should have a short, descriptive message that summarizes the change
Commits should never contain:
- Credentials, private keys, or other sensitive information
- Generated files
- Debugging files

## Secure Coding Standards
Contributions should always follow secure coding practices, which include, but are not limited to:
- Validation of any input
- Proper out of bounds checking
- Variable initialization and scope management
- Pointer validation
- Memory management
- Error handling
- No hardcoded credentials or sensitive information

As a baseline, one should refer to the SEI CERT C and C++ Secure Coding Standards.

### Testing
Before submitting a pull request, ensure that the code passes all automated tests.
This may include:
- Testing on the exact same hardware
- Running applicable unit tests
- Using the configured static analysis and linting tools
- Scanning for secrets
- Dependency scanning for known vulnerabilities

### Automated Checks
The project uses the following test suite for its automated checks:
| Tool | Description |
| --- | --- |
| GitLeaks | Scans for secrets in the codebase |
| OSV-Scanner | Scans for known vulnerabilities |
| Cppcheck | Static analysis and linting tool for C/C++ code |
| Arduino Linter | Lints for the specific Arduino project structure |

Configured and validated checks are the only enforced requirements.
Any proposed changes to existing checks must be approved by the maintainers.
Same applies to any new checks added in the future.

## Pull Request Requirements
All changes should always be submitted as a pull request
Pull requests should include the following:
- Description of the change
- Reason for the change
- Test cases or documentation of the testing that was performed
- Known issues or limitations
- Any relevant evidence or additional context where necessary

## Merge Requirements
A pull request is only able to be merged once:
- Required automated checks pass
- Required correctness and security checks pass
- Any comments or feedback is addressed and resolved
This process should be followed strictly at all times
If for any reason an exception needs to be made, the reviewer must provide a valid reason, explain why, and be approved by the maintainers separately

## Correctness Review Checklist
For the correctness review, the following checklist should be followed:
- Changes within the pull request satisfy the intended requirements
- Implementation behaves as expected
- Relevant tests are include, or have been updated to reflect the changes
- No existing functionality has been broken by the changes
- Code follows the established coding style and conventions
- Required automated tests pass

## Security Review Checklist
For the security review, the following checklist should be followed:
- Changes do not introduce known security vulnerabilities
- Sensitive information / keys are not exposed
- Input handling / verification is robust
- Array bounds / memory operations are safe
- Changes to encryption / firmware signing are documented and separately reviewed
- New / updated dependencies are documented and checked for known vulnerabilities, properly patched if so

## Dependency Management
If the need arises for a new or updated dependency, the following steps should be taken:
  - Document its name and version (if applicable)
  - Explain its use case (if applicable)
  - Review its maintenance status (if applicable)
  - Check for any known vulnerabilities
  - Update dependency list
  - Ensure all checks still pass
It's encouraged with new dependencies to always follow this policy and be able to be automatically scanned for vulnerabilities
A manual review may be substituted in if the dependency is unable to be automatically scanned for a compelling reason

## Additional Information on Security
For additional information on security, please see the [security policy](SECURITY.md).
