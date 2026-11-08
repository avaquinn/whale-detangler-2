# Security Policy

## Supported Versions
Security updates are provided for the latest stable version only
| Version | Supported |
|---------|-----------|
|Current development version|Yes|
|Old versions|No|

## Vulnerability Reporting
If you discover a vulnerability, please report it privately to the project maintainers.
Make sure to include:
  - A detailed description of the vulnerability
  - Steps to reproduce the issue
  - Potential impact
  - Suggested fix
Please do not include any sensitive information in the report.

## Vulnerability Report Response
Maintainers will access submitted reports and determine the appropriate response from there.
Any confirmed vulnerability will be patched in the next release after a review process and testing.
The response process may include:
  - Report validation
  - Severity and impact assessment
  - Affected components / firmware versions
  - Developing or reviewing a fix
  - Testing and documentation of the fix

## Secrets Management
Sensitive information should never be committed to the repository.
Examples of sensitive information include:
- Developer credentials
- Cryptographic keys
- Firmware signing keys
- Any other sensitive material
Thus, secrets should be stored within a secrets management system and have limited access.
If a secret is compromised, it should be rotated immediately and access should be revoked.

## Firmware Integrity
A signature is used to verify the integrity of the firmware.
All private signing keys should be protected and not exposed.
Verification is to be performed by a trusted device-side process.

## Dependencies
All dependencies should be reviewed for any known vulnerabilities or security issues.
If found, known vulnerabilities should be investigated, prioritized, and patched before further use.
Any new dependency should adhere to the project's security policy and be reviewed by the security team.
This is taken care of through automated scanning technologies, which any new dependency should adhere to.
If for any reason a new dependency cannot be scanned, a manual review process should be substituted in its place.
If an exception is being requested or considered, a separate review process should be initiated.

## Security Exceptions
Generally, all security defects should be addressed and resolved immediately.
However, if an exception is being requested or considered, the following should be submitted:
  - Description of the exception
  - Reason for the exception
  - Any potential impact or risk
  - Possible compensating controls
  - Expiration or reassessment date
Once submitted, the exception will be reviewed by the security team and will be approved accordingly.
All exceptions must be reassessed periodically to ensure they are still valid.

## DevSec Policy
Contributors should adhere to the project's security policy and review all code changes for any potential security issues.
In addition, the guidelines layed out in [CONTRIBUTING.md](CONTRIBUTING.md) should be followed.
