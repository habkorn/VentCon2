# CRA Supplier Due Diligence Questionnaire

<!-- extract:head-start -->
## Extraction record

| Field | Value |
| --- | --- |
| Source file | `sec/AalbertsChecklists/templates/CRA- Supplier Due Diligence Questionnaire - template Sep 2026.docx` |
| SHA256 | `a2e6895f515fcf26a937a4bd5b685c3d1f96582903cc8f338d457e4172fe552a` |
| Author | Un-named |
| Created | 2026-09-07 23:00:00+00:00 |
| Modified | 2026-09-15 10:32:00+00:00 |
| Extracted | 2026-10-02 |
| Tool | tools/docx_extract.py, python-docx |
| Scope | paragraphs: 27, tables: 7, numbered paragraphs: 0, images: 1 |
| Formatting | headings follow the source paragraph styles (Title, Heading 1, Heading 2); clause numbers are reconstructed from the numbering definition |
| Header text | Aalberts - internal |
| Footer text | Page 2 of 2 |
| Not reproduced | company logo (image1.jpg), page layout, page number fields |
| Answers | Filled in from the VENTREX dossier on 2026-10-02, marked in bold. Template text is unchanged. Regenerate the extract with tools/docx_extract.py and re-apply with tools/aalberts_fill.py if needed. |
| Note | Extract of the source document, English as in the source. The source file remains the reference. |

<!-- extract:head-end -->

# Supplier Due Diligence Checklist & Questionnaire

EU Cyber Resilience Act — Third-Party Software, Firmware & Component Due Diligence

## Purpose and how to use this document

This questionnaire supports due diligence on third-party software, firmware, and hardware components under Article 13(5) of the Cyber Resilience Act (CRA). Under the CRA, the manufacturer of the finished product remains responsible for the cybersecurity of all integrated components, including supplier-provided software and firmware using a CE-marked or certified component does not, on its own, transfer that responsibility.

Complete one copy of this questionnaire per supplier per component. Attach completed questionnaires, together with any supporting evidence received (SBOMs, certificates, vulnerability disclosure policies), to the product's CRA technical documentation file. Sections 1–6 below mirror the structure agreed for group-wide use across Aalberts Group.

Priority: components used in products with confirmed EU market placement should be assessed first, given the reporting obligations applying from 11 September 2026 and full conformity obligations applying from 11 December 2027.

## Section 1 — Supplier and Component Identification

**The measures referenced in this document, P1.1 to P5.2, are described in the annex VentCon2_Measures_Reference_2026-10-02.docx, delivered with this package.**

Establishes exactly which component, version, and product this assessment covers.

| Question | Supplier response |
| --- | --- |
| Supplier legal name | **VENTREX, part of the Aalberts group.** |
| Supplier country of establishment | **Austria.** |
| Component name and version + model number | **VentCon2 valve control unit, firmware 2.7.4 on ESP32-S3 (Arduino Nano ESP32); prototype hardware, no series model number yet** |
| Component type (software, firmware, hardware with embedded software, cloud/SaaS service) | **Hardware with embedded software and a local web interface; no cloud service.** |
| Can the component be considered to be open source? | Fully open source (specify in SBOM) / Partially open source (specify in SBOM) / Proprietary i.e. Closed source (code not openly shared under a license) <br>**No. The firmware is proprietary, it is not published under a free and open-source licence.** |
| Intended use of the digital elements of the component, as per the manufacturer | **Regulate the outlet pressure in a fuel line, actuate the shut-off valve, monitor the pressure and close the valve on implausible readings; operation through a local WLAN access point.** |
| Aalberts product(s) / product line(s) into which this component is integrated | **The VentCon2 pressure control system itself; prototype operation in the own test rig, series not decided.** |
| Aalberts company responsible for this integration | **VENTREX.** |
| Brief description of the component's function within the product | **PID control of the outlet pressure with 10 ms sampling and a 1 kHz control loop, PWM actuation of the solenoid valve at 2 kHz and 14 bit, pressure measurement 0 to 10 bar, web interface with live chart, settings in LittleFS.** |
| Date this component was first integrated / expected integration date | **In operation in the own test rig since 2026 (firmware 2.7.4); series integration not scheduled.** |
| Committed Support Period (min 5 years!) | **Not committed yet. The CRA baseline is at least five years or the expected use time; measure P4.4 open, end date to be defined and published with the user information.** |

## Section 1a — CRA role classification

Confirms the supplier has analysed its own role under the CRA.

| Question | Supplier response |
| --- | --- |
| Has supplier assessed its own role designation under the CRA? | **Yes, documented in sec/CRA_Compliance_Checklist.md section 2.** |
| If so, what role does it consider itself to have: | Manufacturer / Authorised representative (of a manufacturer outside the EU) / Importer / Distributor / Open-source software steward <br>**Manufacturer within the meaning of Art. 3(13) CRA. No importer, distributor or authorised representative is involved, because VENTREX places the product under its own name and no third-party distribution is planned.** |

## Section 2 — Security ownership

Confirms the supplier has a named point of contact and a defined response commitment for security matters.

| Question | Supplier response |
| --- | --- |
| Does the supplier have a designated security contact or team (e.g. PSIRT)? | **Partly: VENTREX Engineering department is the entry point; the single point of contact required by Art. 13(17) CRA is planned with measures P4.8 and P4.5.** |
| Contact details for reporting a suspected vulnerability or incident | **VENTREX Engineering department**; today the development contact and the serial console are the only channels |
| Committed acknowledgement time upon receipt of a vulnerability report | **Not committed yet. The CRA reporting duties of 24 h, 72 h, 14 days and one month apply from placing the product on the EU market; the internal process is measure P5.1 and the reporting access is measure P5.2, both resting.** |
| Committed time to a substantive response or remediation plan | **Not committed yet; remediation is handled case by case. Measure P4.5 open.** |
| Does the supplier have a published vulnerability disclosure or coordinated disclosure policy? (attach if yes) | **No, measure P4.5 open.** |

## Section 3 — Vulnerability handling

Assesses how actively the supplier tests for and manages vulnerabilities in the component.

| Question | Supplier response |
| --- | --- |
| Does the supplier conduct regular security testing of the component (e.g. static/dynamic analysis, penetration testing)? Describe. | **No regular security testing. One manual security analysis of the web interface exists (2026-08-27, findings R1 to R9) and the code is reviewed during development; update tests are part of measure P4.3.** |
| Has the component had any publicly disclosed or CVE-registered vulnerabilities in the past 24 months? List, if any. | **None registered. All findings so far are internal and documented as risks R1 to R9.** |
| How and how quickly will the supplier notify Aalberts if a new vulnerability affecting this component is identified? | **No committed process yet. Notification through the development contact; the formal process is measure P5.1, resting until the market decision.** |
| Does the supplier provide security updates separately from feature updates where technically feasible? | **Not established. Firmware and file system are uploaded separately with development tooling only; the release process is measure P4.3.** |
| Will the supplier support coordinated fixes if a vulnerability is identified jointly, including sharing fixes upstream for open-source elements? | **In principle yes. Reporting vulnerabilities in third-party components to their maintainers (Art. 13(6) CRA) is listed as an open duty in the master and not yet a defined process.** |

## Section 4 — Bill of Materials and Transparency

Establishes whether the supplier can identify what is inside their own component, including bundled open-source elements.

| Question | Supplier response |
| --- | --- |
| Can the supplier provide a software bill of materials (SBOM) for this component, in a machine-readable format (e.g. CycloneDX, SPDX)? | **No SBOM yet, measure P4.2 open.** |
| Does the component include third-party libraries? If so, can a full list be provided? | **Yes, at top level: ArduinoJson 7.4.1, Adafruit ADS1X15 2.5.0, Arduino PID Library, littlefs, the Arduino core for ESP32 with ESP-IDF and FreeRTOS, and Chart.js vendored as data/chart.min.js. The list is available from platformio.ini, a full automatically generated list does not exist yet.** |
| Is the SBOM kept up to date on each new release, and will Aalberts receive updated versions? | **No SBOM exists yet, so no update cycle; part of measure P4.2.** |
| Are there any known unsupported or end-of-life dependencies within the component? | **Not assessed yet; to be determined with measure P4.2.** |

## Section 5 — Support Commitment

Confirms the supplier's support period aligns with Aalberts' own CRA support-period obligations (minimum five years).

| Question | Supplier response |
| --- | --- |
| For how long does the supplier commit to providing security updates for this component version? | **Not committed yet; intended at least five years or the expected use time, measure P4.4.** |
| What happens to security support when this component version reaches end-of-life — is a supported upgrade path provided? | **Not defined yet; measure P4.4 for the support period and P4.3 for the update path.** |
| Does the supplier's committed support period meet or exceed Aalberts' required support period for the product it is integrated into? | **Not determined yet. The target is to meet the Aalberts requirement of at least five years; the end date will be published in the user information as required by Annex II No. 7 CRA.** |
| Are security updates provided free of charge for the duration of the support commitment? | **Intended yes, in line with the CRA; no update delivery exists yet, measure P4.3.** |
| What advance notice will the supplier give before ending support for this component? | **Not defined yet. The end date of the support period has to be stated in the user information, measure P4.4 and P4.8.** |

## Section 6 — Certification and Documentation

Gathers existing conformity evidence the supplier can hand over for use in Aalberts' own technical documentation.

| Question | Supplier response |
| --- | --- |
| Is the component already CE marked? If so, under which regulation(s)/directive(s)? | **No. The product is not placed on the EU market (prototype in the own test rig, planned market India). CE marking is planned for the case of EU placement; the declaration of conformity and the CE marking are measure P4.7, the CRA route is internal control (module A), the RED route is open in measure P1.5.** |
| Has the component been assessed against any recognised standard (e.g. IEC 62443, ETSI EN 303 645, ISO/IEC 27001)? Attach evidence. | **ETSI EN 303 645 is used as a technical benchmark in the dossier (clauses 5.1-1 for passwords and 5.4-1 for stored credentials) without certification. IEC 62443 and ISO/IEC 27001 have not been assessed.** |
| Can the supplier provide technical documentation suitable for inclusion in Aalberts' CRA conformity file upon request? | **Partly. The dossier holds the master checklist, 21 measure plans and the risk and evidence analysis; the technical documentation in the structure of Annex VII CRA is measure P4.6 and still open.** |
| Is the supplier aware of, and does it acknowledge, its own obligations as a manufacturer, importer, or distributor under the CRA? | **Yes. Applicability, product classification as a standard product, the 26 obligations of the project list and the deadlines are documented in sec/CRA_Compliance_Checklist.md; the market status is not placed on the EU market, so the obligations rest until the first supply into the Union.** |

### Sign-off

Completed by (name, role, entity name): **Product owner, VENTREX (Aalberts group company)**

Date: **2026-10-02**

Outcome: (to be verified by legal counsel)

**Outcome: Accepted with conditions / follow-up required.** Gaps identified and listed in the checklist; no material risk for the current status, because the product is not placed on the EU market. The formal outcome remains with legal counsel.

☐ Component accepted — no material gaps

☐ Accepted with conditions / follow-up required

☐ Escalate — material risk identified

<!-- extract:appendix-start -->
## Reviewer comments from the source document

- Niels Ketelaar (2026-09-14T16:46:00Z): Confusing to me. Should it be LIST the type of security testing or describe the testing process?

<!-- extract:appendix-end -->
