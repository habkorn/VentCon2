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
| Extracted | 2026-10-06 |
| Tool | tools/docx_extract.py, python-docx |
| Scope | paragraphs: 27, tables: 7, numbered paragraphs: 0, images: 1 |
| Formatting | headings follow the source paragraph styles (Title, Heading 1, Heading 2); clause numbers are reconstructed from the numbering definition |
| Header text | Aalberts - internal |
| Footer text | Page 2 of 2 |
| Not reproduced | company logo (image1.jpg), page layout, page number fields |
| Note | Extract of the source document, English as in the source. The source file remains the reference. |

<!-- extract:head-end -->

# Supplier Due Diligence Checklist & Questionnaire

EU Cyber Resilience Act — Third-Party Software, Firmware & Component Due Diligence

## Purpose and how to use this document

This questionnaire supports due diligence on third-party software, firmware, and hardware components under Article 13(5) of the Cyber Resilience Act (CRA). Under the CRA, the manufacturer of the finished product remains responsible for the cybersecurity of all integrated components, including supplier-provided software and firmware using a CE-marked or certified component does not, on its own, transfer that responsibility.

Complete one copy of this questionnaire per supplier per component. Attach completed questionnaires, together with any supporting evidence received (SBOMs, certificates, vulnerability disclosure policies), to the product's CRA technical documentation file. Sections 1–6 below mirror the structure agreed for group-wide use across Aalberts Group.

Priority: components used in products with confirmed EU market placement should be assessed first, given the reporting obligations applying from 11 September 2026 and full conformity obligations applying from 11 December 2027.

## Section 1 — Supplier and Component Identification

Establishes exactly which component, version, and product this assessment covers.

| Question | Supplier response |
| --- | --- |
| Supplier legal name |  |
| Supplier country of establishment |  |
| Component name and version + model number |  |
| Component type (software, firmware, hardware with embedded software, cloud/SaaS service) |  |
| Can the component be considered to be open source? | Fully open source (specify in SBOM) / Partially open source (specify in SBOM) / Proprietary i.e. Closed source (code not openly shared under a license) |
| Intended use of the digital elements of the component, as per the manufacturer |  |
| Aalberts product(s) / product line(s) into which this component is integrated |  |
| Aalberts company responsible for this integration |  |
| Brief description of the component's function within the product |  |
| Date this component was first integrated / expected integration date |  |
| Committed Support Period (min 5 years!) |  |

## Section 1a — CRA role classification

Confirms the supplier has analysed its own role under the CRA.

| Question | Supplier response |
| --- | --- |
| Has supplier assessed its own role designation under the CRA? |  |
| If so, what role does it consider itself to have: | Manufacturer / Authorised representative (of a manufacturer outside the EU) / Importer / Distributor / Open-source software steward |

## Section 2 — Security ownership

Confirms the supplier has a named point of contact and a defined response commitment for security matters.

| Question | Supplier response |
| --- | --- |
| Does the supplier have a designated security contact or team (e.g. PSIRT)? |  |
| Contact details for reporting a suspected vulnerability or incident |  |
| Committed acknowledgement time upon receipt of a vulnerability report |  |
| Committed time to a substantive response or remediation plan |  |
| Does the supplier have a published vulnerability disclosure or coordinated disclosure policy? (attach if yes) |  |

## Section 3 — Vulnerability handling

Assesses how actively the supplier tests for and manages vulnerabilities in the component.

| Question | Supplier response |
| --- | --- |
| Does the supplier conduct regular security testing of the component (e.g. static/dynamic analysis, penetration testing)? Describe. |  |
| Has the component had any publicly disclosed or CVE-registered vulnerabilities in the past 24 months? List, if any. |  |
| How and how quickly will the supplier notify Aalberts if a new vulnerability affecting this component is identified? |  |
| Does the supplier provide security updates separately from feature updates where technically feasible? |  |
| Will the supplier support coordinated fixes if a vulnerability is identified jointly, including sharing fixes upstream for open-source elements? |  |

## Section 4 — Bill of Materials and Transparency

Establishes whether the supplier can identify what is inside their own component, including bundled open-source elements.

| Question | Supplier response |
| --- | --- |
| Can the supplier provide a software bill of materials (SBOM) for this component, in a machine-readable format (e.g. CycloneDX, SPDX)? |  |
| Does the component include third-party libraries? If so, can a full list be provided? |  |
| Is the SBOM kept up to date on each new release, and will Aalberts receive updated versions? |  |
| Are there any known unsupported or end-of-life dependencies within the component? |  |

## Section 5 — Support Commitment

Confirms the supplier's support period aligns with Aalberts' own CRA support-period obligations (minimum five years).

| Question | Supplier response |
| --- | --- |
| For how long does the supplier commit to providing security updates for this component version? |  |
| What happens to security support when this component version reaches end-of-life — is a supported upgrade path provided? |  |
| Does the supplier's committed support period meet or exceed Aalberts' required support period for the product it is integrated into? |  |
| Are security updates provided free of charge for the duration of the support commitment? |  |
| What advance notice will the supplier give before ending support for this component? |  |

## Section 6 — Certification and Documentation

Gathers existing conformity evidence the supplier can hand over for use in Aalberts' own technical documentation.

| Question | Supplier response |
| --- | --- |
| Is the component already CE marked? If so, under which regulation(s)/directive(s)? |  |
| Has the component been assessed against any recognised standard (e.g. IEC 62443, ETSI EN 303 645, ISO/IEC 27001)? Attach evidence. |  |
| Can the supplier provide technical documentation suitable for inclusion in Aalberts' CRA conformity file upon request? |  |
| Is the supplier aware of, and does it acknowledge, its own obligations as a manufacturer, importer, or distributor under the CRA? |  |

### Sign-off

Completed by (name, role, entity name): _______________________________________________

Date: _______________

Outcome: (to be verified by legal counsel)

☐ Component accepted — no material gaps

☐ Accepted with conditions / follow-up required

☐ Escalate — material risk identified

<!-- extract:appendix-start -->
## Reviewer comments from the source document

- Niels Ketelaar (2026-09-14T16:46:00Z): Confusing to me. Should it be LIST the type of security testing or describe the testing process?

<!-- extract:appendix-end -->
