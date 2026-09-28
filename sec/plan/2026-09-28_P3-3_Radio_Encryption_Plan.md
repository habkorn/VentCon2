# Plan: Encryption and Authentication on the Radio Layer (P3.3)

Rolle: Umsetzungsplan (Detaildokument, Skelett)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: P3.3
CRA-Bezug: Annex I, Teil I, 1.8 (Pflicht-Nr. 5)
Risikobezug: Funkebene ohne eigene Authentifizierung auf Anwendungsebene
Risikoanalyse: `../analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md`
Abhaengigkeiten: P2.2 (AP-Passwort), P1.1 (Sitzungsauthentifizierung)
Datum: 2026-09-28
Hinweis: Skelett. Abschnitt 2 ist verifiziert, die Abschnitte 3 bis 6 sind zu fuellen.

## 1. Objective

Secure the wireless path between browser and device, or document the accepted residual risk if
transport encryption is not feasible on this platform.

## 2. Current State (verified 2026-09-28)

- `WiFi.softAP(ap_ssid, ap_password, 1, 0, NetworkConfig::MAX_CLIENTS)`
  (`src/WebHandlers.cpp`, `initializeWiFiAP`): WPA2-PSK, fixed channel 1, up to two clients
  (`MAX_CLIENTS = 2` in `src/Constants.h`).
- All control traffic is plain HTTP. No TLS stack is used: no `WiFiServerSecure`, no `Update.h`,
  no mbedTLS or BearSSL usage found anywhere in `src/`.
- The AP password is hardcoded today; P2.2 makes it configurable, which changes the shared-secret
  situation but not the transport security.
- The radio link is encrypted by WPA2-PSK between client and AP, so the exposure is mainly inside
  the client device and in any browser-to-device trust assumptions (for example captive portal
  interception, malicious client on the same AP with the shared password).

## 3. Design (TODO)

Options to evaluate, each with memory cost and usability impact:

a) Keep WPA2-PSK, rely on the P1.1 session authentication as the compensating control, and document
   the residual risk in the risk assessment (P4.1).
b) HTTPS with a self-signed certificate, generated on first boot, trust-on-first-use in the browser.
   Check flash and RAM cost against the memory-efficient chunked rendering in `handleRoot()`.
c) WPA3-SAE or WPA2-Enterprise for the softAP: verify what the ESP32-S3 supports in AP mode.

## 4. Implementation Steps (TODO)

## 5. Verification (TODO)

## 6. Open Items

- The decision depends on what is actually exposed (setpoint, measured pressure, calibration data).
- Certificate validity handling on a device without a real-time clock.
- Interaction with the browser UI: no mixed content, no failed certificate warnings for end users.
