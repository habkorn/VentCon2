# Plan: Authenticate All Mutating Web Endpoints (P1.1, P1.2, minimal P2.1)

Date: 2026-08-29
Status: Plan; implementation pending (revised 2026-08-31: first-boot provisioning added)
Reference: `sec/2026-08-27_Webinterface_Sicherheitsanalyse.md` (findings 1, 4, 5)
Companion report: `sec/2026-08-29_Web_Auth_How_It_Works.md`

## 1. Objective

Introduce authentication for all mutating endpoints of the web interface and close
the two directly coupled holes:

- P1.1: login page with password; session required for every mutating endpoint.
- P1.2: `/set` and `/resetPID` move to POST; CSRF protection on all mutations.
- P2.1 (minimal): `/settings.json` and `/default.json` are no longer served over HTTP,
  because the password hash will live in `settings.json`.
- First-boot provisioning: no universal default password. The operator must set the
  control password before the device becomes operational (CRA Annex I, Part I,
  2(b)/(d); ETSI EN 303 645 5.1-1: passwords "unique per device or defined by the
  user").

Confirmed decisions (user, 2026-08-29; revised 2026-08-31):

- Password stored as salted PBKDF2-HMAC-SHA256 in `settings.json`, user-defined at
  first boot.
- No universal default password: the device ships in a provisioning state and the
  operator must set a password before any control endpoint answers.
- Session: RAM-only, one shared token, idle timeout 30 minutes, invalid on reboot.
- CSRF: POST-only mutation, `SameSite=Lax` session cookie, `X-Requested-With` header.
- Documents in English; code comments in German (project convention).

## 2. Current State

- `WebHandler::setupRoutes()` (`src/WebHandlers.cpp`, line 822) registers `/set`
  (line 829) and `/resetPID` (line 837) without an HTTP method, so both accept GET.
- `/api/slider-limits` (lines 841/845) and `/api/chart-settings` (lines 849/853)
  already split GET (read) and POST (write); only the session guard and CSRF header
  are missing on the POST branches.
- No login, session, cookie, or rate-limit code exists anywhere in `src/`.
- `handleRoot()` (line 346) streams the page in PROGMEM chunks; `WebContent.h`
  (lines 18 to 22) aggregates the chunks, so a login chunk fits the pattern.
- The page's JavaScript (`src/WebContentScript.h`) centralizes all calls:
  `/values` line 747, `/set` lines 1265 and 1450, `/resetPID` lines 1305 and 1469,
  `/api/slider-limits` lines 1391, 1564, 1650, `/api/chart-settings` lines 1416, 1782.
- `onNotFound` (line 876) passes any path to `handleFileRead()` (line 216), which
  serves `/settings.json` and `/default.json` to anyone.
- `SettingsHandler` already provides mutex-protected `update()` / `snapshotTo()` and
  JSON load/save, which the password field reuses.
- No provisioning state exists: the device boots straight into the control page with
  no credential configured.
- Current version: 2.7.3 (`VERSION`, `CHANGELOG.md`).

## 3. Design

### 3.1 Password storage

- New fields in `SettingsHandler`:
  - `char auth_password_salt[33]` (32 hex chars + NUL), random per password set.
  - `char auth_password_hash[65]` (64 hex chars + NUL).
  - `bool auth_provisioned` (persisted). False = provisioning state, true = the
    operator has set a password and the device is operational.
- `bool checkPassword(const String& plain)`: PBKDF2-HMAC-SHA256 via mbedtls with
  the stored salt and `AuthConfig::PBKDF2_ITERATIONS`, hex encoding, constant-time
  comparison with the stored hash. When `auth_provisioned` is false, compare
  against a fixed dummy hash so the response timing does not reveal whether the
  device is provisioned.
- `bool setPassword(const String& plain)`: generates a fresh random salt
  (`esp_random()`), derives the hash, sets `auth_provisioned = true`, persists.
  Used by the provisioning flow and later by the runtime password change feature
  (P2.2).
- `load()`: when the hash is missing or empty, `auth_provisioned` stays false.
  There is no universal default password that unlocks control functions; an
  unprovisioned device only serves the setup page (section 3.6).
- `save()`: serialize the three new fields into `settings.json`.

A salted key-derivation function is used instead of raw SHA-256 because raw
SHA-256 is fast and therefore weak against offline dictionary attack: a leaked
hash would be crackable at high speed. PBKDF2 with a random salt and a high
iteration count makes each guess expensive. Argon2 is stronger but needs far more
RAM than the ESP32 has, so PBKDF2-HMAC-SHA256 is the state-of-the-art-appropriate
choice for this device (see also section 7).

### 3.2 Session

- State in `WebHandler` (RAM only): `sessionActive`, `sessionToken` (from
  `esp_random()`), `sessionLastSeenMs`, `loginAttempts`, `lockoutUntilMs`.
- `handleLogin()` (POST): 429 during lockout; `checkPassword()` decides; success
  sets the cookie `session=<hex>; HttpOnly; SameSite=Lax; Path=/` and answers JSON
  `{"success":true}` (the page reloads; no server redirect needed).
- `handleLogout()` (POST): clears cookie (`Max-Age=0`) and token.
- `bool requireAuth()`: parses the `Cookie` header, compares the token in constant
  time, refreshes the idle timestamp; expired or missing token means false.
- Guarded handlers answer 401 when `requireAuth()` fails; the page shows the login.

### 3.3 CSRF

- Route changes: `/set` and `/resetPID` registered as `HTTP_POST` only.
- Every mutating handler rejects requests without header
  `X-Requested-With: fetch` (403).
- Frontend switches to `fetch(url, {method:'POST', body: params})` with
  `Content-Type: application/x-www-form-urlencoded` and the CSRF header.
- Defense model: SameSite=Lax cookie blocks cross-site POSTs carrying the session;
  the custom header blocks cross-site POSTs without a session (no CORS preflight
  support on the ESP32).

### 3.4 JSON file blocking

- In `onNotFound`, before `handleFileRead()`, refuse `/settings.json` and
  `/default.json` (404). Log the attempt. Full path sanitization stays a separate
  measure.

### 3.5 Login rate limiting

- 5 failed attempts: 60 s lockout (constants in `AuthConfig`). Logging via
  `LOG_I(CAT_NETWORK, ...)` for success, failure, and lockout.

### 3.6 First-boot provisioning (no universal default password)

The device must never ship with a credential that unlocks control functions.
Instead it ships in a provisioning state, so the password is "defined by the user"
(ETSI EN 303 645 5.1-1). This satisfies CRA Annex I, Part I, 2(b) (secure by
default) and 2(d) (authentication). A universal default password such as a
`DEFAULT_PASSWORD` constant is explicitly rejected.

State machine (persisted via `auth_provisioned`):

```
                    ┌──────────────────────────┐
      boot ───────▶ │ provisioning (no hash)   │
                    └─────────────┬────────────┘
                                  │ POST /setup (valid password)
                                  ▼
                    ┌──────────────────────────┐
                    │ operational (hash set)   │
                    └─────────────┬────────────┘
                                  │ POST /logout, or session expiry
                                  ▼
                    ┌──────────────────────────┐
                    │ locked (login required)  │
                    └──────────────────────────┘
```

- **Provisioning state** (`auth_provisioned == false`):
  - `handleRoot()` streams the setup page (`WebContentSetup.h`) instead of the
    control page. The login page is not shown; there is nothing to log in to yet.
  - `POST /setup` is the only mutating endpoint that answers. It reads the control
    password (`password`, `password2`) and the AP password (`ap_password`,
    `ap_password2`), validates 8 to 64 chars and equality for each, and rejects
    the submission when the control password equals the AP password. On success it
    calls `setPassword()` for the control password and writes the AP credentials
    (P2.2), then answers `{"success":true}`; the page reloads into the login page.
  - Every other endpoint, including the read-only `/values`, answers 403 so no
    control or telemetry data leaks before provisioning. Static assets
    (chart.js, logo, manual) stay open so the setup page can render.
- **Operational state** (`auth_provisioned == true`): the normal login flow
  (section 5.1) applies; the setup page is gone.
- **Factory reset** (restore defaults) clears `auth_password_hash` and sets
  `auth_provisioned = false`, returning the device to the provisioning state, never
  to a universal password (CRA Annex I, Part I, 2(b): the reset state must itself
  be secure by default).
- **Forgotten password recovery**: no remote recovery path exists (by design) - a
  remote recovery path would be a backdoor that defeats authentication. Recovery
  is physical:
  1. A dedicated credentials-only reset action (separate button/hold combination)
     clears `auth_password_hash` and sets `auth_provisioned = false`, returning
     the device to the provisioning state without touching PID gains, sensor
     calibration, or chart settings.
  2. The operator sets a fresh password on the setup page.
  A full factory reset remains the fallback and clears all settings as today.
  Document the recovery in the user manual.

Password policy for v1: minimum 8 characters, maximum 64, printable ASCII, the two
fields must match. Complexity rules (character classes) are a follow-up.

The setup page collects two separate fields: the control password (hashed) and the
AP password (stored recoverably). They are deliberately distinct credentials:

- The AP password is shared with every device that joins the Wi-Fi and must be
  stored recoverably (the WPA2 handshake needs the passphrase), so it cannot be
  hashed.
- The control password gates a safety-relevant function (setpoint, PID gains,
  calibration) and is therefore hashed and never stored recoverably.

Using one shared password would collapse these two boundaries: anyone who ever had
the Wi-Fi password would also have full control authority, and the control
credential would sit in a recoverable plaintext field (see section 7).

## 4. Implementation Steps

Phase A - `src/Constants.h`, `src/SettingsHandler.h/.cpp`:
1. `AuthConfig` namespace: `MIN_PASSWORD_LEN` (8), `MAX_PASSWORD_LEN` (64),
   `SALT_LEN` (16 bytes), `PBKDF2_ITERATIONS` (value TBD by measurement, target
   ~100 ms per derivation), `SESSION_TIMEOUT_MS` (1800000), `MAX_LOGIN_ATTEMPTS`
   (5), `LOCKOUT_MS` (60000), `CSRF_HEADER_NAME`, `CSRF_HEADER_VALUE`. No
   `DEFAULT_PASSWORD` constant exists.
2. `auth_password_salt`, `auth_password_hash`, and `auth_provisioned` fields,
   `checkPassword()`, `setPassword()`, PBKDF2-HMAC-SHA256 helper (mbedtls
   `mbedtls_pkcs5_pbkdf2_hmac`; verify API name against the installed
   framework-arduinoespressif32 version), load/save integration. `load()` never
   fills the hash with a default; it only reads what was saved and leaves
   `auth_provisioned` false when the hash is empty.

Phase B - `src/WebHandlers.h/.cpp`, new `src/WebContentLogin.h`,
`src/WebContentSetup.h`, `src/WebContent.h`:
3. Session and lockout state plus helpers in `WebHandler`.
4. `handleLogin()`, `handleLogout()`, `handleSetup()`, `requireAuth()`, CSRF header
   check helper, and `isProvisioned()`.
5. `setupRoutes()`: POST-only for `/set`, `/resetPID`; add `/login`, `/logout`,
   `/setup`; guard the two API POST branches.
6. `handleRoot()`: when not provisioned, stream `HTML_SETUP`; otherwise stream
   `HTML_LOGIN` when `requireAuth()` fails and the control page when it succeeds.
   Add `WebContentSetup.h` and `WebContentLogin.h` to `WebContent.h`.
7. Provisioning gate: while `auth_provisioned` is false, every handler except
   `handleSetup()`, the static assets, and `handleRoot()` answers 403.

Phase C - `src/WebContentScript.h`:
8. POST conversion for `/set` (lines 1265, 1450) and `/resetPID` (lines 1305,
   1469); CSRF header on all mutating fetches (also lines 1391, 1416, 1650, 1782).
9. Login form submit, logout, and 401 handling (show login, stop polling).
10. Setup form submit (control password + confirmation, AP password +
    confirmation; client-side length, equality, and distinctness checks before
    POST) and handling of `403` from non-setup endpoints while in the provisioning
    state.

Phase D - Data, docs, version:
11. `data/default.json` and `data/settings.json`: add `auth_password_salt` (empty
    string), `auth_password_hash` (empty string), and `auth_provisioned` (false) so
    a fresh device boots into the provisioning state.
12. `CHANGELOG.md` entry under `[Unreleased]`; bump `VERSION` (2.7.3 to 2.8.0,
    pending confirmation); run the existing build with `inject_version.py`.
13. Update `sec/2026-08-27_Webinterface_Sicherheitsanalyse.md`: mark P1.1 and P1.2
    done, P2.1 partially done, extend the decisions section (first-boot
    provisioning, no universal default password), and add a note under finding 1
    that the single-client limit (P3.2) does not replace authentication
    (cross-reference this report, Part 2.4).

Phase E - Verify (section 5), then finalize the report's line references.

## 5. Verification

### 5.1 Static

- `pio run` completes without warnings or errors.
- Review checklist: no GET mutation remains; every mutating route calls
  `requireAuth()` and the CSRF check; cookie flags correct; no password plaintext
  anywhere; German comments.

### 5.2 Functional (curl against 192.168.4.1)

| Check | Command (expected result) |
|---|---|
| GET no longer mutates | `curl http://192.168.4.1/set?sp=5` -> 405 |
| POST without session | `curl -X POST http://192.168.4.1/set -d sp=5` -> 401 |
| POST without CSRF header | `curl -X POST -H "Cookie: session=<token>" http://192.168.4.1/set -d sp=5` -> 403 |
| POST fully valid | cookie + `X-Requested-With: fetch` -> 200, `settings.json` updated |
| Login wrong password | `curl -X POST http://192.168.4.1/login -d password=x` -> 401 |
| Login lockout | repeat 5 times -> 429 |
| Login correct | -> 200, `Set-Cookie` contains `HttpOnly` and `SameSite=Lax` |
| Fresh device root | `curl http://192.168.4.1/` -> setup page; `/values` -> 403 |
| Setup mismatch | `POST /setup password=a1234567&password2=b1234567` -> 400 |
| Setup too short | `POST /setup password=short&password2=short` -> 400 |
| Setup valid | `POST /setup password=<p>&password2=<p>` -> 200; next `/` -> login page |
| JSON blocking | `curl http://192.168.4.1/settings.json` and `/default.json` -> 404 |
| Logout | POST `/logout` -> cookie cleared, next `/set` -> 401 |

### 5.3 Session behavior

- Idle timeout: with a temporarily shortened `SESSION_TIMEOUT_MS`, a guarded
  request after the timeout returns 401.
- Reboot: after reset, the old cookie is rejected and the login page appears.

### 5.4 Browser flows

- Save button, PID reset, restore defaults, slider limit editor, chart settings:
  all work unchanged after login.
- Boot with a pre-existing `settings.json` without the auth field (upgrade from
  v2.7.3): page loads into the setup page; the operator must set a password before
  the control page appears.
- Fresh device and factory reset: setup page; no endpoint answers before
  provisioning.

### 5.5 Security regression

- Repeat the three attack paths from the security analysis (direct access, CSRF
  simulation, configuration download) and confirm all are blocked.

### 5.6 Stress

- Rapid repeated save clicks; 10 minute soak; check task stack high-water marks via
  the existing task info output (pattern from the P3.1 plan).

### 5.7 Rollback

- Every change is a separate git commit; the previous firmware image remains
  flashable.

## 6. Risks

- **Cookie handling in the WebServer library**: set via
  `webServer.sendHeader("Set-Cookie", ...)`; verify the header survives the
  chunked response path.
- **mbedtls API variance**: `mbedtls_pkcs5_pbkdf2_hmac` signature differs between
  core versions; confirm against the installed framework before coding.
- **KDF latency**: PBKDF2 with a high iteration count blocks the network task for
  roughly the target ~100 ms per login attempt. Measure the iteration count so the
  target fits the lockout design; the lockout already caps the number of attempts
  per window.
- **JS argument encoding**: POST body must be form-urlencoded and the content type
  set, otherwise `webServer.arg()` stays empty.
- **Shared session**: both allowed clients share one token; acceptable for the
  documented single-operator use case.
- **Forgotten password**: no remote recovery path; recovery is a physical
  credentials-only reset (separate button/hold) that does not wipe calibration.
  Document the recovery clearly in the user manual.
- **Provisioning state abuse**: the unauthenticated `/setup` endpoint must reuse
  the login attempt counter and lockout, or an attacker can consume it.
- **401 handling in the page**: the polling loop must stop and show the login page,
  otherwise the page retries continuously.

## 7. Decisions (confirmed)

- Scope: P1.1 + P1.2 + minimal P2.1.
- Documents in English; code comments in German.
- Web password hashed with salted PBKDF2-HMAC-SHA256 (random salt, high iteration
  count) in `settings.json`, user-defined at first boot; no universal default
  password (CRA Annex I, Part I, 2(b)/(d); ETSI EN 303 645 5.1-1). Argon2 rejected:
  too much RAM for the ESP32.
- Control password and AP password remain two separate credentials: the AP password
  is shared with every Wi-Fi client and stored recoverably, so sharing it with the
  safety-relevant control function would collapse the transport-access and
  control-authorization boundaries (EN 303 645 5.5-5, 5.6-7; CRA Annex I 2(d)).
- RAM-only session, shared token, 30 minute idle timeout, invalid on reboot.
- CSRF: POST-only mutation + `SameSite=Lax` cookie + `X-Requested-With` header.
- Read-only endpoints (`/values`, GET APIs, static files) remain open.
- Forgotten-password recovery: dedicated physical "credentials-only" reset that
  clears only the password (keeps PID gains, calibration, chart settings); full
  factory reset remains the fallback.
- First-boot setup sets the control password and the AP password on one combined
  setup page (shared with P2.2) and rejects identical control/AP passwords.
