# Plan: Runtime-Configurable AP Password (P2.2)

Date: 2026-08-31
Status: Plan; implementation pending
Reference: `sec/2026-08-27_Webinterface_Sicherheitsanalyse.md` (finding 2)
Prerequisite: `sec/2026-08-29_Web_Auth_Plan.md` (P1.1 and P1.2 must be implemented first)

## 1. Objective

Replace the hardcoded, publicly documented AP password `ventcon12!` with a
runtime-configurable value stored in `settings.json`. The AP SSID is included in
the change for consistency. The web endpoint for changing the credentials is
protected by the session guard introduced in P1.1.

Finding 2 from the security analysis: the password is printed in the source code,
the documentation, and the user manual. Any device in radio range that has ever
seen those documents can join the WLAN and reach all endpoints. A runtime-
configurable password is the prerequisite for replacing the default before
deployment; it is not a fix on its own and must be combined with P1.1.

Confirmed decisions:

- AP SSID and AP password stored as plain text in `settings.json`.
- No universal default AP password (CRA Annex I, Part I, 2(b)/(d); ETSI EN 303 645
  5.1-1). A fresh device ships in the provisioning state and the operator must set
  the AP password before the device becomes operational.
- The provisioning state is shared with the web-login provisioning of P1.1: one
  setup page sets both the control password and the AP password (see
  `sec/2026-08-29_Web_Auth_Plan.md`, section 3.6).
- Password change triggers a deferred WiFi restart (see section 3.4).
- Endpoint is guarded by the session check from P1.1 and the CSRF header from P1.2.
- Documents in English; code comments in German (project convention).

## 2. Current State

- `NetworkConfig::AP_PASSWORD[] = "ventcon12!"` in `src/Constants.h` (line 17),
  compiled directly into the firmware binary.
- `NetworkConfig::AP_SSID[] = "VENTCON_AP"` in the same namespace (line 14).
- WiFi is started with these constants in `WebHandler::initWiFi()` (exact line TBD
  during implementation; search for `WiFi.softAP(`).
- `SettingsHandler` has no AP credential fields; `settings.json` has no
  corresponding keys today.
- No web endpoint exists for changing WLAN credentials.
- No provisioning state exists: the AP starts secured with the public default and
  the control interface is immediately reachable.

## 3. Design

### 3.1 Storage

Two new fields in `SettingsHandler`:

```cpp
char ap_ssid[33];       // max 32 chars + NUL (IEEE 802.11 SSID limit)
char ap_password[64];   // max 63 chars + NUL (WPA2 passphrase limit; min 8)
```

- `load()`: when the key is missing, the device is in the provisioning state: the
  AP starts with a unique SSID and no password (open), and only the setup page is
  reachable until the operator sets a password (section 3.6). When the key is
  present and 8 to 63 chars, the AP starts secured with it.
- `save()`: serialize both fields into `settings.json` under keys `"ap_ssid"` and
  `"ap_password"`.

The AP password is stored recoverably (plaintext) because the WPA2 handshake needs
the passphrase; hashing is impossible. It is therefore a "sensitive security
parameter" and must be stored securely (ETSI EN 303 645 5.4-1): persist the
credential pair in an encrypted NVS partition (NVS encryption, key in eFuse)
instead of `settings.json`, so the stored credential is not trivially readable
from the flash. Never hard-code the credential in source (5.4-3).

`NetworkConfig::AP_PASSWORD` is removed from `src/Constants.h`; there is no
fallback default password. `NetworkConfig::AP_SSID` remains as the base of the
unique provisioning SSID (e.g. `VENTCON_AP-XXXX` with the last MAC octets).

### 3.2 New endpoint: POST `/api/wifi-settings`

Handler `handleWiFiSettings()` in `src/WebHandlers.h/.cpp`:

- **GET branch**: returns `{"ap_ssid":"...","ap_password_set":true}` — the
  password value is never echoed back; only a boolean indicates whether a
  password is configured. (The password is stored recoverably in encrypted NVS;
  never returning it limits exposure from XSS or CSRF leaks.)
- **POST branch**: guarded by `requireAuth()` and the CSRF header check (same
  pattern as `/api/slider-limits` POST).
  - Reads `ap_ssid` and `ap_password` from the form body.
  - Validates: SSID 1–32 chars; password 8–63 chars; both must be printable ASCII.
  - Writes under the settings mutex; calls `settings->save()`.
  - Schedules a deferred WiFi restart (see 3.4).
  - Returns `{"success":true,"restart_pending":true}` so the page can show a
    warning.

Registered in `setupRoutes()` alongside the existing API endpoints.

### 3.3 `initWiFi()` update

Replace:

```cpp
WiFi.softAP(NetworkConfig::AP_SSID, NetworkConfig::AP_PASSWORD, ...);
```

with a snapshot read:

```cpp
SettingsSnapshot cfg;
settings->snapshotTo(cfg);
WiFi.softAP(cfg.ap_ssid, cfg.ap_password, ...);
```

`SettingsSnapshot` gains `char ap_ssid[33]` and `char ap_password[64]` fields to
carry the values atomically. In the provisioning state (empty password) the AP
starts open with the unique SSID (section 3.6).

### 3.4 Deferred WiFi restart

`WiFi.softAP()` with new credentials cannot be called from inside the HTTP handler
because the network task is already inside the WebServer dispatch loop. Restarting
the AP mid-request disconnects the client before the response is sent.

Approach: set a boolean flag `wifiRestartPending` in `WebHandler`. The network task
loop checks this flag after each `webServer.handleClient()` call. When set, it
calls `WiFi.softAPdisconnect(true)` followed by `WiFi.softAP(...)` with the new
credentials, then clears the flag. A short delay (100 ms) is inserted to let the
final response reach the browser.

The browser-side warning tells the user that the connection will drop and they must
rejoin with the new password. The page stops polling (`/values`) until reconnect.

### 3.5 Frontend changes (`src/WebContentScript.h`)

- New settings section in the web UI for WLAN credentials (alongside the existing
  PID and sensor sections).
- GET `/api/wifi-settings` on page load to show the current SSID and whether a
  custom password is set.
- POST on save, same pattern as the slider limits editor, with the CSRF header.
- On `{"restart_pending":true}`: show a banner: "WiFi will restart. Reconnect to
  the new SSID and reload this page."

### 3.6 Provisioning state (no universal default AP password)

A fresh device must never start the AP secured with a publicly known password.
Instead, when `settings.json` has no `ap_password`, the device boots into a
provisioning state:

- The AP starts with a unique SSID (`VENTCON_AP-XXXX`, last two MAC octets) and no
  password (open).

The unique SSID exists only during provisioning, because that is the one window
where the AP is open:

- **Identification**: with several units nearby, each broadcasting the same
  `VENTCON_AP`, the operator cannot tell which one is theirs in the Wi-Fi list.
- **Anti-spoofing**: an open AP with a shared name is indistinguishable from a
  neighbor's unit or a malicious lookalike. Uniqueness removes the ambiguity while
  there is no password to authenticate the AP itself.
- **Transient**: after provisioning the operator sets their own SSID, so the
  unique default exists only for the first-boot window.

The MAC address is public (broadcast in every beacon), so using it for the SSID is
safe; it must never be used to derive a password or any other secret.

- The web interface serves the setup page only; all control endpoints answer 403
  (this gate is implemented by P1.1, see `sec/2026-08-29_Web_Auth_Plan.md`,
  section 3.6). The single setup page sets the control password AND the AP
  password together.
- The shared `/setup` handler writes the AP credentials (together with the control
  password) and triggers the deferred WiFi restart (section 3.4). After the
  restart the AP comes up secured with the new password and a normal SSID.
- A factory reset clears both credential fields, returning the device to the
  provisioning state, never to `ventcon12!`.

`ventcon12!` is removed from the firmware entirely; it survives nowhere in code or
data files.

### 3.7 Secure NVS partition (implementation notes)

The AP credential pair (`ap_ssid`, `ap_password`) is persisted to an encrypted NVS
partition instead of `settings.json`. On the ESP32-S3 (Arduino Nano ESP32) this is
done with the ESP-IDF NVS encryption API:

- **Key storage**: without full flash encryption, the NVS encryption keys must live
  in eFuse, not in a plaintext `nvs_keys` partition (a plaintext key partition
  would defeat the encryption). Use the eFuse + HMAC scheme
  (`CONFIG_NVS_SEC_KEY_PROTECT_USING_HMAC`, `esp_efuse`/`esp_hmac`): the XTS keys
  are derived at runtime inside the HMAC peripheral and never leave eFuse.
- **SoC check**: confirm ESP32-S3 eFuse/HMAC support for NVS key protection before
  coding. If unavailable, full flash encryption becomes the fallback (see risk).
- **Partition table**: the `nvs` partition already exists in
  `board_build.partitions = default_16MB.csv`; with eFuse HMAC keys no additional
  `nvs_keys` partition is required.
- **Init**: the Arduino core starts NVS unencrypted (`nvs_flash_init`) at boot.
  Replace or precede this with `nvs_flash_secure_init()` /
  `nvs_flash_secure_init_partition()` so the credential namespace is encrypted from
  first access.
- **Arduino API caveat**: the `Preferences` wrapper does not expose encryption; call
  the IDF `nvs_flash_secure_*` / `nvs_*` APIs directly (`#include "nvs_flash.h"`).
- **Key generation**: burn the HMAC key to eFuse once, at manufacturing or in a
  guarded first-boot step. The eFuse block is one-time-programmable, so this must
  be part of the production flow, not an accidental runtime step.
- **OTA impact**: OTA updates must preserve the NVS partition; document this in the
  OTA procedure.

## 4. Implementation Steps

Phase A - `src/Constants.h`, `src/SettingsHandler.h/.cpp`, `src/WebHandlers.h/.cpp`:
1. Remove `AP_PASSWORD` from `NetworkConfig` in `src/Constants.h`; keep
   `AP_SSID` as the base of the unique provisioning SSID. No default password
   constant remains.
2. Add `ap_ssid`, `ap_password` to `SettingsHandler` and `SettingsSnapshot`;
   implement load/save with the provisioning-state fallback (section 3.1). The
   credential pair is persisted to an encrypted NVS namespace instead of
   `settings.json` (section 3.1).
3. Update `initWiFi()` to read from the snapshot.
4. Add `wifiRestartPending` flag and deferred restart logic in the network loop.
5. Implement `handleWiFiSettings()` (GET + POST); register route.

Phase B - Secure storage at rest:
6. Store the AP credential pair in an encrypted NVS partition (NVS encryption with
   keys in eFuse, section 3.7), not in `settings.json`. Full flash encryption is
   out of scope for this measure (see section 7). The recoverable AP passphrase is
   then not trivially readable from the flash (ETSI EN 303 645 5.4-1).

Phase C - `src/WebContentScript.h`:
7. GET on load; credential form; POST on save; restart warning banner.

Phase D - Data and version:
8. `data/default.json` and `data/settings.json`: no `ap_ssid`/`ap_password` keys
   (the credential pair lives in encrypted NVS). `ventcon12!` appears nowhere.
9. `CHANGELOG.md` entry; version bump (coordinate with P1.1/P1.2 release or
   separate minor bump, TBD).

## 5. Verification

### 5.1 Static

- `pio run` without warnings.
- No literal `"ventcon12!"` anywhere in `src/` or `data/`. `"VENTCON_AP"` appears
  only as the SSID base in `Constants.h`.
- `handleWiFiSettings()` POST branch calls `requireAuth()` and the CSRF check.
- Password value is never included in any GET response.

### 5.2 Functional (curl against 192.168.4.1 after P1.1 is active)

| Check | Expected result |
|---|---|
| GET `/api/wifi-settings` without session | 401 |
| GET with session | 200; `"ap_password_set": false` (no password configured) |
| POST new SSID/password without CSRF header | 403 |
| POST with invalid password (< 8 chars) | 400 |
| POST valid credentials | 200; `"restart_pending": true` |
| AP restarts; join with new password; GET `/` | control page loads |
| Firmware reboot after change | new credentials still active (loaded from flash) |

### 5.3 Fallback

- Flash-erase + firmware upload (no `settings.json`): device boots into the
  provisioning state (unique open SSID, setup page only); the operator sets the
  AP password before normal operation.
- `settings.json` present but no `ap_password` key (upgrade from v2.7.3):
  device boots into the provisioning state (same as above).

## 6. Risks

- **Deferred restart timing**: if the response does not reach the browser before
  the AP restarts, the browser shows a network error. The 100 ms delay mitigates
  this; increasing it to 500 ms is a safe fallback.
- **Plain-text storage**: the AP password is stored recoverably (the WPA2 handshake
  needs the passphrase, so hashing is impossible). Per ETSI EN 303 645 5.4-1 it is
  a sensitive security parameter and must be stored securely: persist it in an
  encrypted NVS partition (NVS encryption with eFuse keys). The file download block
  from minimal P2.1 removes the HTTP exposure; NVS encryption removes the physical
  extraction exposure.
- **No confirmation field**: a typo in the new password locks the operator out.
  Mitigation: require the password to be typed twice in the form, compared in JS
  before submission. Alternatively, keep serial access as a recovery path
  (the firmware already accepts `set_wifi_password` via UART if needed; add this
  command if it does not exist yet — confirm during implementation).
- **Open provisioning AP**: during provisioning the AP has no password (always on
  while unprovisioned, per the confirmed decision). The window is short (until the
  operator sets the password) and only the setup page is reachable, but an
  attacker in range could set a password first and lock out the owner. Accepted
  residual risk for v1; the credentials-only physical reset (section 3.6) is the
  recovery path if that happens.
- **Session invalidation after restart**: the session token lives in RAM and is lost
  on the WiFi restart. The user must log in again after reconnecting. This is
  correct behavior and must be explained in the UI.

## 7. Decisions (confirmed 2026-08-31)

- Store the AP password recoverably (not hashed) because the WPA2 handshake needs
  the passphrase; protect it at rest in an encrypted NVS partition (NVS encryption
  with eFuse keys) per ETSI EN 303 645 5.4-1, never hard-coded in source (5.4-3).
  Full flash encryption is out of scope for this measure.
- SSID included in scope (not only the password).
- Browser-side reconnect guidance rather than an automatic redirect (impossible
  after the AP SSID changes).
- Provisioning entry: the open provisioning AP is always on while the device is
  unprovisioned (no physical button press required).
- First-boot setup uses a single combined page that sets the control password and
  the AP password together (shared with P1.1).
- Forgotten-password recovery: dedicated physical "credentials-only" reset (see
  P1.1 plan, section 3.6); serial recovery command not required.
- The setup page rejects identical control and AP passwords (enforced separation).
