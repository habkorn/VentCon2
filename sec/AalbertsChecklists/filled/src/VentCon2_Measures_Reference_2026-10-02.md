# VentCon2: Measures Referenced in the Aalberts Answers

Annex to the CRA product maturity checklist and to the supplier due diligence questionnaire, reference date 2026-10-02.

<!-- annex:head-start -->
| Field | Value |
| --- | --- |
| Source | sec/CRA_Compliance_Checklist.md, the single status source in the VENTREX dossier (section 7 and section 8). Measures P1.1 to P5.2, 21 items, and findings R1 to R10. Duty numbers refer to the obligation list 1 to 26 in section 6 of the same file. |
| Market trigger | the duties arise with the placing of the product on the Union market (Art. 2(1), Art. 3(21) and (22) CRA, Recital 15). As long as the product is distributed in third countries only, no duties arise from the CRA or from the RED cyber provisions; implementation is voluntary then. Measures marked with "rests until the market decision" are not being implemented today, measures marked with "active in prototype operation" are. |
| Revision | 2026-10-06: findings R1 to R10 added; the risk reference is split into the columns Finding and CRA duty. |
| Closing | Status as documented on 2026-10-06. The master checklist remains the single status source; this annex is an extract of it. |
| Tool | tools/measure_reference.py, generated from section 7 and section 8 of the master and composed into VentCon2_Measures_Reference_2026-10-02.docx. |
<!-- annex:head-end -->

## Measures P1.1 to P5.2

| ID | Measure and scope | Status | Finding | CRA duty |
| --- | --- | --- | --- | --- |
| P1.1 | **Authentication for all mutating endpoints** Login page and first-start provisioning for every endpoint that changes state. | In progress, active in prototype operation | R1 | 2 |
| P1.2 | **POST migration and CSRF protection** Move /set and /resetPID from GET to POST and protect them against cross-site requests. | In progress, active in prototype operation | R4 | 6 |
| P1.3 | **Limit the sensor calibration to the physical sensor range** Reject calibration values above the sensor range (sensor_maxP <= SENSOR_MAX_BAR). | In progress, active in prototype operation | R3 | 9 |
| P1.4 | **Hard upper limits for PID gains** Cap the PID gains so a wrong configuration cannot drive the valve to its limits. | Open | R6 | 4 |
| P1.5 | **Clarify the EN 18031 conformity route and the CE route** Radio equipment duties of the RED, valid until 2027-12-11. | Open, rests until the market decision | none (RED norm duty) | RED-1, RED-2 |
| P2.1 | **Path sanitisation and no configuration over HTTP** Sanitise handleFileRead() and stop serving the JSON configuration over HTTP. | In progress, partly designed (JSON lock) | R5 | 3 |
| P2.2 | **Access point password configurable at runtime** Remove the password that is fixed in the source and make it configurable. | In progress, active in prototype operation | R2 | 1, 10 |
| P2.3 | **Rate limiting and save debouncing** Limit request rates and debounce settings writes to protect the flash memory. | Open | R7 | 8 |
| P3.1 | **Mutex for settings access** Serialise concurrent access to the settings structure. | Done, in v2.7.3 | R9 | no Annex I point (robustness) |
| P3.2 | **Passive rejection of new clients instead of dropping all** A further client is rejected instead of disconnecting the existing connection. | Done, in v2.7.3 | R8 | 8 |
| P3.3 | **Encryption or authentication at the radio level** Protect the wireless link itself, today the access point has no own authentication. | Open | R10 | 5 |
| P4.1 | **Document the cybersecurity risk assessment** Written risk assessment for the product, the interim basis today is the security analysis. | Open, rests until the market decision | none (standard duty) | 16 |
| P4.2 | **SBOM and due diligence for third-party components** Machine-readable bill of materials for Arduino core, Arduino-ESP32, LittleFS, chart.js and the further libraries. | Open, rests until the market decision | none (standard duty) | 11, 22 |
| P4.3 | **Signed OTA updates and update tests** Implement the signed update mechanics and test the update path. | Open, rests until the market decision | none (standard duty) | 7, 13, 15 |
| P4.4 | **Support period and end date** Define the support period, at least five years or the expected service life if shorter, and document the end date. | Open, rests until the market decision | none (standard duty) | 20 |
| P4.5 | **Vulnerability management and disclosure policy** Coordinated disclosure policy plus a security contact and the remediation process. | Open, rests until the market decision | none (standard duty) | 12, 14 |
| P4.6 | **Technical documentation in the structure of Annex VII** Assemble the technical documentation required by the CRA. | Open, rests until the market decision | none (standard duty) | 17 |
| P4.7 | **EU declaration of conformity and CE marking** Declaration of conformity in the structure of Annex V and the CE marking. | Open, rests until the market decision | none (standard duty) | 18, 19 |
| P4.8 | **User information in the structure of Annex II** Update the user information with the security relevant content required by the CRA. | Open, rests until the market decision | none (standard duty) | 21 |
| P5.1 | **Reporting process for exploited vulnerabilities and severe incidents** Internal process for the reporting duties of 24 hours, 72 hours, 14 days and one month. | Open, rests until the market decision | none (reporting duty) | 23, 25, 26 |
| P5.2 | **Access to the ENISA Single Reporting Platform and the competent CSIRT** Register the reporting access required for the reporting duties. | Open, rests until the market decision | none (reporting duty) | 24 |

## Findings R1 to R10

| ID | Finding | Measure | Status |
| --- | --- | --- | --- |
| R1 | No authentication on the control interface, every endpoint that changes state is reachable without a login. | P1.1 | In progress, active in prototype operation |
| R2 | Access point password fixed in the source code and published in the documentation. | P2.2 | In progress, active in prototype operation |
| R3 | Sensor calibration can be changed over the web interface, the firmware then underestimates the real pressure and the over-pressure shutdown does not trip. | P1.3 | In progress, active in prototype operation |
| R4 | State changes over HTTP GET, so a foreign page can change control parameters. | P1.2 | In progress, active in prototype operation |
| R5 | Arbitrary file access on LittleFS, the JSON configuration with calibration and PID gains is served over HTTP. | P2.1 | In progress, partly designed (JSON lock) |
| R6 | PID gains can be raised without a limit over the web interface. | P1.4 | Open |
| R7 | Flash wear and denial of service, every settings write goes synchronously to LittleFS. | P2.3 | Open |
| R8 | Third client connection dropped all clients and restarted the access point. | P3.2 | Done, in v2.7.3 |
| R9 | Missing synchronisation of the settings accesses between network and control task. | P3.1 | Done, in v2.7.3 |
| R10 | Radio layer without its own authentication, the control traffic runs as plain HTTP over the wireless link. | P3.3 | Open |
