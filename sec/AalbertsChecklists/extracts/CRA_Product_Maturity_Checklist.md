# CRA Product Maturity Checklist

<!-- extract:head-start -->
## Extraction record

| Field | Value |
| --- | --- |
| Source file | `sec/AalbertsChecklists/templates/CRA product maturity checklist - Sep 2026.docx` |
| SHA256 | `fab3c5d4c605845bf352f511f925d5e1fc4efa589572a6dc84eb9ca5ca8aeb46` |
| Author | Un-named |
| Created | 2026-09-15 10:34:00+00:00 |
| Modified | 2026-09-15 10:34:00+00:00 |
| Extracted | 2026-10-06 |
| Tool | tools/docx_extract.py, python-docx |
| Scope | paragraphs: 40, tables: 2, numbered paragraphs: 2, images: 1 |
| Formatting | headings follow the source paragraph styles (Title, Heading 1, Heading 2); clause numbers are reconstructed from the numbering definition |
| Header text | Aalberts - internal |
| Footer text | Page 2 of 2 |
| Not reproduced | company logo (image1.jpg), page layout, page number fields |
| Note | Extract of the source document, English as in the source. The source file remains the reference. |

<!-- extract:head-end -->

# CYBER RESILIENCE ACT (CRA) PRODUCT COMPLIANCE CHECKLIST

## essential cybersecurity requirements*

## Instruction

## Fill out one checklist per product that has been identified as being in scope of the CRA (see Aalberts CRA Memo).

## The product compliance under the CRA is "risk-based", which means that the strength of below listed security controls must scale with what could go wrong for that specific product — a highly connected product handling sensitive data needs stronger protections (e.g., encryption, strict access control, active monitoring) than a simple, offline, low-data device.

## To validate product’s compliance, you should first map out realistic attack scenarios and their potential impact (likelihood × severity), then check whether the below safeguards that are in place are proportionate to that risk level. For examples of potential impacts to consider, see Appendix 1 below.

## This checklist assumes that all your products in scope of the CRA fall under the "regular" products category under the CRA; should you have any doubts, please refer to the list of important and critical products here.

Product: __________ Owner: __________ Date: __________<br>Part I: Product cybersecurity

| Compliance assessment<br>Y / N or N/A (+justification) | Category | Safeguard (details) |
| --- | --- | --- |
|  | Risk-based security |  |
|  |  | Risk-based design: The product’s security measures match its identified cybersecurity risks (see Appendix 1 below). |
|  |  | No known exploitable vulnerabilities: None are known when the product is released or placed on the market. |
|  |  | Secure by default: Default settings are secure, and the product can be reset to its original state. Any different setup for a tailor-made business product is expressly agreed with the business customer. |
|  | Updates and access |  |
|  |  | Security updates: Vulnerabilities can be fixed through security updates. |
|  |  | Automatic updates where appropriate: Updates are installed automatically within an appropriate period and enabled by default, with clear notification and easy options to opt out or temporarily postpone them. |
|  |  | Access protection: Authentication, identity and access controls prevent unauthorised access. |
|  |  | Access alerts: Possible unauthorised access is detected and reported. |
|  | Data and system protection |  |
|  |  | Confidentiality: Relevant stored and transmitted data is protected, including through current encryption methods where appropriate. |
|  |  | Integrity: Data, commands, software and settings are protected against unauthorised changes, and corruption is detected and reported. |
|  |  | Data minimisation: The product processes only the data needed for its intended purpose. |
|  |  | Availability: Essential and basic functions remain available, or can recover, after an incident, including denial-of-service attacks. |
|  |  | No harmful network impact: The product minimises disruption to services provided by connected devices or networks. |
|  | Secure design and user control |  |
|  |  | Limited attack surface: External interfaces, access points and unnecessary functions are minimised. |
|  |  | Incident impact limited: Security controls reduce the possible impact of successful attacks. |
|  |  | Security logging and monitoring: Relevant internal activity, including access to or changes to data, services and functions, is recorded and monitored. Users can opt out. |
|  |  | Secure data removal and transfer: Users can easily and permanently delete all data and settings. Any transfer to another product or system is secure. |
|  | Secure development methodologies |  |
|  |  | Languages are chosen for their secure properties: e.g. memory safety, alternatively bounds checking tools are deployed |
|  |  | Program analysis: static and dynamic analysis, type checking, embedded OS allows for reference monitoring, taint checking and similar techniques |
|  |  | Formal verification |
|  |  | Compilation certification or verifiable builds: the code as shipped can be verified against the source code |

Part II: Vulnerability handling

| Assessment<br>Yes / No /<br>N/A (+justification) | Safeguard (details) |
| --- | --- |
|  | Vulnerability and component records: Vulnerabilities and product components are identified and documented. |
|  | Software Bill of Materials: An SBOM is maintained in a commonly used, machine-readable format and includes at least top-level dependencies. |
|  | Prompt remediation: Vulnerabilities are fixed without delay, based on risk. |
|  | Separate security updates: Where technically possible, security updates are provided separately from feature updates. |
|  | Separate security updates: Where technically possible, security updates are provided separately from feature updates. |
|  | Regular security testing: Effective security tests and reviews are performed regularly. |
|  | Information about fixed vulnerabilities: Once an update is available, information is publicly shared on the vulnerability, affected product, impact, severity and user remediation steps. Publication may be delayed where justified by security risk until users can apply the patch. |
|  | Coordinated disclosure policy: A coordinated vulnerability disclosure policy is in place and followed. |
|  | Reporting channel: A contact address or similar channel is available for reporting vulnerabilities in the product and its third-party components. |
|  | Secure update delivery: Updates are distributed securely and allow vulnerabilities to be fixed or reduced in a timely manner, automatically where applicable. |
|  | Fast, free and explained: Security updates are provided without delay, free of charge, and with clear user guidance, unless different terms are agreed for a tailor-made business product. |

Overall result: ☐ Compliant ☐ Gaps identified ☐ Further assessment required<br>Key gaps / evidence: ______________________________________________________

*please refer to Annex I of the EU CRA

APPENDIX 1 – Product risk assessment**

Step 1 – Identify the product's context: list how the product connects, what data it handles, and who uses it (consumers, businesses, critical infrastructure).

Step 2 – Map realistic attack scenarios: think through plausible ways the product could be attacked or misused, based on its connectivity, data sensitivity, and attack surface (see list of examples below)

Step 3 – Evaluate impact and likelihood: for each scenario, estimate how likely it is and how severe the consequences would be (e.g., safety, financial, privacy, reputational, or legal harm).

Step 4 – Match safeguards to risk level: ensure the security controls in place (encryption, access control, monitoring, etc.) are proportionate to the assessed risk — stronger protections for higher-risk products, simpler protections for lower-risk ones. Use above checklist for list of essential cybersecurity requirements.

Examples of impacts you should consider when assessing cyber-attack scenarios on your product:

1. Safety impact

Could the attack cause physical harm to users of your product or bystanders, such as a malfunctioning medical device or industrial control system causing injury?

2. Sensitive data confidentiality impact

Could sensitive or personal data (e.g., health records, credentials, financial information) be exposed?

3. Data integrity impact

Could the attack alter, corrupt, or falsify data, commands, or configurations in a way that leads to incorrect operation or decision-making?

4. Availability/operational impact

Could the attack disrupt or disable core functions of the product, causing downtime, service interruption, or loss of essential functionality (e.g., via denial-of-service)?

5. Financial impact

Could the attack result in direct monetary loss for Aalberts group company (manufacturer) or for the product’s user, such as through fraud, ransomware payments, regulatory fines, or business interruption costs?

6. Reputational impact

Could the incident damage Aalberts group company’s (manufacturer’s) or customer's reputation, eroding customer trust and market position, even if the technical harm is limited?

7. Environmental impact

Could the attack cause environmental harm, for example by disrupting industrial or energy systems that control emissions, waste, or hazardous materials?

8. Legal and regulatory impact

Could the attack trigger non-compliance with laws such as the CRA, GDPR, or sector-specific regulations, exposing Aalberts group company to fines, investigations, or mandatory disclosures?

9. Cascading/network impact

Could the compromised product affect other connected devices, systems, or networks, causing a wider chain reaction of failures (e.g., in IoT ecosystems or supply chains)?

10. National security/critical infrastructure impact

Could the product's compromise affect critical infrastructure (e.g., energy, water, transport) with implications for public safety or national security, particularly relevant for products used in essential sectors?

**Notes on executing a product risk assessment:

- For CRA wording refer to Article 13(2) of the EU CRA

- If any of the above categories 1, 2, 3, 9 and 10 are in play, do consider applying an in-depth methodology for cybersecurity threat modelling for embedded systems, e.g. the MITRE EMB3D Threat Model, https://emb3d.mitre.org/
