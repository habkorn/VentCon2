# How the Web Interface Endpoints and Authentication Work in VentCon2

Rolle: Grundlagenerklaerung (Hintergrundwissen, kein Umsetzungsplan)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: P1.1, P1.2, P2.1
CRA-Bezug: Annex I, Teil I, 1.2, 1.4 und 1.10 (Pflicht-Nr. 2, 3 und 6)
Risikobezug: R1, R4, R5
Risikoanalyse: `../analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md`
Umsetzungsplan: `../plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md`
Datum: 2026-08-29
Inhalt: Part 1 introduces the web basics needed to follow the rest.
Parts 2 to 4 describe the firmware as it is today (v2.7.3). Part 5 describes the
authentication mechanism agreed on 2026-08-29, which is not yet implemented.
Part 6 lists the boundaries.

---

## Part 1 - Web Basics for Non-Web-Developers

The whole web interface is built from a handful of simple ideas. This chapter
explains them first; the rest of the document builds on them.

### 1.1 What happens when you open a page

A browser talks to a server in pairs: a **request** and a **response**.

```
Browser (your phone)                          ESP32 web server
──────────────────────                        ──────────────────
"What should I show?                          (looks up the answer)
 I would like  GET / "
───────────────────────────────request──────────────▶

                                              "Here is an HTML page."
◀──────────────────────────────response───────────────

Browser draws the page.
```

A request has three parts that matter here:

- **The path**: the part after the address, e.g. `/set` or `/values`.
- **The method**: GET or POST (see 1.3).
- **Optional parameters**: extra data, e.g. `?sp=5.0`.

### 1.2 What is an endpoint?

An **endpoint** is a path the server knows how to answer. In VentCon2, each endpoint
is one entry in a lookup table: "when someone asks for THIS path, call THIS function."

The table is built in `WebHandler::setupRoutes()` (`src/WebHandlers.cpp`, line 822).
Every `webServer.on(...)` call adds one row to the table:

```
webServer.on("/values", [this](){ this->handleValues(); });
//            └path┘      └ function to call ─────────────────┘
```

So "the endpoint /values" means: the path `/values` plus the function
`handleValues()` that runs when it is requested. That is all an endpoint is.

### 1.3 GET vs POST

Both are ways a browser asks a server for something. The difference is intent:

| | GET | POST |
|---|---|---|
| Purpose | "Give me data" | "Change something / deliver data" |
| Parameters travel | Visible in the URL (`/set?sp=5`) | Invisibly in the request body |
| Triggered by | Typing a URL, clicking a link, loading an image, `fetch('/values')` | Form submission, `fetch(url, {method:'POST', ...})` |
| Side effects | Should have none | May change state |

Two consequences follow:

1. **Anything that changes state must be POST.** A GET can be fired by accident (link
   prefetch, URL autocomplete, a page reload, an embedded image).
2. **CSRF is much harder against POST.** A malicious web page can trigger a cross-site
   GET without restrictions (the attack is described in Part 2.3), but a cross-site
   POST is restricted by the browser's SameSite cookie rules (Part 4).

Today `/set` and `/resetPID` change state via GET, which is exactly finding 4 of the
security analysis. Part 2 shows why that is dangerous.

### 1.4 The journey of a request through the code

Following the example of saving a new setpoint:

```
Browser                      WiFi stack              WebServer library           WebHandler
───────                      ──────────              ─────────────────           ──────────
1. fetch("/set?sp=5")
   builds an HTTP GET
   for path /set
                             sends the request
                             to 192.168.4.1
                                                     receives it
                                                     looks up "/set" in the
                                                     table built by
                                                     setupRoutes()
                                                     finds: handleSet
                                                                                2. handleSet() runs
                                                                                   reads sp from the
                                                                                   request (arg)
                                                                                   validates 0..10
                                                                                   writes settings
                                                                                   saves to flash
                                                     sends "OK" back
                             radio transmit
3. browser receives
   "OK", shows snackbar
```

The interesting code is tiny:

- **Reading parameters**: `webServer.arg("sp")` returns the value that followed `sp=`
  in the URL (`src/WebHandlers.cpp`, line 482).
- **Answering**: `webServer.send(200, "text/plain", "OK")` sends the response
  (`src/WebHandlers.cpp`, line 621).

Everything else (radio, TCP, HTTP parsing, the route table) is done by the Arduino
`WebServer` library that the firmware gets for free.

---

## Part 2 - The Problem: Why Endpoints Need Authentication

Now that the basic vocabulary is in place, this chapter shows what goes wrong when
the endpoints answer everyone.

### 2.1 What authentication means

The ESP32 runs a small web server. Whenever your browser asks for something, the
server answers. Today the server answers **everybody**, without ever asking who is
asking.

**Authentication** is the question "who are you?", followed by "are you allowed to do
this?" (that second question is called **authorization**). A login page with a password
is the simplest form: you prove you know the password, and the server then trusts your
requests for a while.

### 2.2 Why this matters for a valve controller

The web interface is the primary control surface for a valve that regulates pressures
up to about 10 bar. Two properties make missing authentication dangerous:

1. **Everything is changeable without a login.** The setpoint, the PID gains, the PWM
   settings, the sensor calibration: one URL call each, no password.
2. **The sensor calibration feeds the safety system.** The emergency shutdown compares
   the measured pressure against a limit that is derived from the calibration values.
   Raising `sensor_maxP` makes the firmware underestimate the real pressure, so the
   shutdown never triggers (finding 3 in the security analysis).

### 2.3 The three attack paths that exist today

**Path A: direct access.**
The access point password is `ventcon12!`, printed in the source code and the manuals.
Any device in radio range can join the WLAN and then call the mutating endpoints
directly.

**Path B: CSRF through the victim's browser.**
The phone or laptop that normally controls the valve is usually online at the same
time (mobile data, second WLAN). If that browser opens a malicious web page, the page
can send requests to `http://192.168.4.1/...` in the background. For simple GET
requests, the browser sends them without asking. The attacker never needs to be in
radio range, and never needs the WLAN password. The victim's own browser is the proxy.
CSRF (Cross-Site Request Forgery) is explained in plain words in Part 4; Part 1.3
already showed why GET requests are so easy to trigger.

**Path C: downloading the configuration.**
`/settings.json` and `/default.json` contain the calibration values and PID gains.
They are plain files on the flash, and the file fallback route serves any file by name.

### 2.4 Why the one-client limit does not help

Since v2.7.3 the access point allows at most `MAX_CLIENTS` (currently 2) simultaneous
clients and rejects additional ones passively (measure P3.2). Does that remove the
problem? No, for three reasons:

1. **Counting is not identifying.** The client limit counts connected devices; it does
   not check who they are. The one allowed slot can just as well be occupied by an
   attacker, because joining the WLAN only requires the public password.
2. **CSRF needs no slot.** In Path B the request comes from the connected device
   itself, so the slot count is irrelevant.
3. **Slots can be freed by force.** WLAN deauthentication frames are unauthenticated
   and trivial to send. An attacker can kick the legitimate device off the network and
   take the free slot.

The client limit is an **availability** measure. It narrows the exposure window, but it
removes none of the attack paths. **Authentication is about identity; the client limit
is about capacity.** Both are needed.

---

## Part 3 - The Endpoints of VentCon2 (Inventory)

All routes are registered in `WebHandler::setupRoutes()` (`src/WebHandlers.cpp`,
line 822). Handlers live in the same file. The JavaScript that calls them lives in
`src/WebContentScript.h` (all page logic is one big PROGMEM script).

| Endpoint | Method today | Handler | What it does | Auth today | Auth planned |
|---|---|---|---|---|---|
| `/` | GET | `handleRoot()` (line 346) | Serves the control page, streamed in chunks | none | login page when not logged in |
| `/set` | any (GET used) | `handleSet()` (line 471) | Applies parameter changes and saves to flash | none | session required, POST |
| `/values` | GET | `handleValues()` (line 608) | JSON snapshot of live values for the chart | none | none (read-only) |
| `/resetPID` | any (GET used) | `handleResetPID()` (line 632) | Resets the PID controller state | none | session required, POST |
| `/api/slider-limits` | GET | `handleSliderLimits()` (line 658) | Returns current slider min/max/step | none | none (read-only) |
| `/api/slider-limits` | POST | `handleSliderLimits()` (line 658) | Changes slider min/max/step | none | session required |
| `/api/chart-settings` | GET | `handleChartSettings()` (line 724) | Returns chart axis settings | none | none (read-only) |
| `/api/chart-settings` | POST | `handleChartSettings()` (line 724) | Changes chart axis settings | none | session required |
| `/setup` | new | new `handleSetup()` | First-boot provisioning: sets the password | - | open |
| `/login` | new | new `handleLogin()` | Checks the password, starts a session | - | open |
| `/logout` | new | new `handleLogout()` | Ends the session | - | open |
| `/chart.min.js` | GET | lambda at line 858 | Serves the chart library from flash | none | none |
| `/Logo.svg` | GET | lambda at line 864 | Serves the logo image | none | none |
| `/VentCon_User_Manual.pdf` | GET | lambda at line 870 | Serves the user manual | none | none |
| anything else | any | `onNotFound` (line 876) to `handleFileRead()` (line 216) | Serves a flash file by its name, else 404 | none | blocks JSON config files |

Note the `Method today` column: `/set` and `/resetPID` are registered **without**
a method (`webServer.on("/set", ...)` at lines 829 and 837), which means the library
accepts every method, including GET. That is the root cause of finding 4.

### 3.1 `/` - the control page

`handleRoot()` (line 346) does not build one big page in memory. It streams the page
in pieces, most of them directly from flash memory (PROGMEM):

```
webServer.sendContent_P(HTML_HEAD);      // page skeleton (WebContentHead.h)
webServer.sendContent_P(CSS_STYLES);     // styling (styles.h)
webServer.sendContent_P(HTML_BODY_START);
inputs.replace("%SP%", ...);             // insert current values into the
webServer.sendContent(inputs);           // input fields (WebContentInputs.h)
webServer.sendContent_P(HTML_SCRIPT);    // all page logic (WebContentScript.h)
```

The parts are `#include`d from `src/WebContent.h` (lines 18 to 22). This is why
adding a login page is cheap: it is just one more PROGMEM chunk that `handleRoot()`
streams instead of the control page when nobody is logged in.

### 3.2 `/set` - applying changes

`handleSet()` (line 471) is the workhorse. The page's save button calls it with all
pending changes at once (e.g. `/set?sp=6.0&kp=6200`). Inside:

1. Every parameter is read with `webServer.arg(...)` and validated against its limits
   (setpoint against `min(sp_limits.max, sensor_max_pressure, 10.0)`, PID gains
   against their slider limits, PWM against the hardware limits, and so on).
2. Valid values are written into the settings object **under the settings mutex**
   (the `settings->update(...)` block, line 475), so the control task on the other
   core never sees a half-written value (see `2026-08-28_Mutex_How_It_Works.md`).
3. Hardware side effects follow outside the lock: PWM resolution and frequency
   reconfiguration, PID re-tuning, and finally `settings->save()`, which writes
   `settings.json` to the flash.

The JavaScript side: `handleSaveClick()` in `WebContentScript.h` (line 1232) collects
the pending changes, builds the URL, and calls `fetch("/set?" + params)` at line 1265.
The restore-defaults flow calls `/set` as well (line 1450).

### 3.3 `/values` - the live data feed

`handleValues()` (line 608) takes an atomic snapshot of all settings
(`settings->snapshotTo(cfg)`, same pattern as the mutex document describes) and
answers with one short JSON text containing the setpoint, gains, filter, PWM, and the
current pressure and PWM readings. The page polls this every second
(`fetch('/values')` at `WebContentScript.h` line 747) to update the chart and the
readouts. It is read-only and stays open.

### 3.4 `/resetPID` - resetting the controller

`handleResetPID()` (line 632) puts the PID controller into manual mode, drives the
valve output to zero, resets the filter to the current reading, re-applies the gains
and limits, and switches back to automatic. The page triggers it from the Reset button
(`fetch('/resetPID')` at `WebContentScript.h` line 1305, and from the restore flow at
line 1469). Because it changes control behavior, it becomes POST-only and guarded.

### 3.5 `/api/slider-limits` - slider ranges

One handler, two roles, split by `webServer.method()`:

- **GET** (line 660): answers with the current min/max/step of the four sliders
  (sp, kp, ki, kd). Read-only.
- **POST** (line 672): changes one slider's limits. The slider name arrives as
  `param`, the new values as `min`, `max`, `step`. After validation (max must exceed
  min, step must be positive) the values are written under the mutex and saved.

The page loads the current limits on startup (`fetch('/api/slider-limits')` at
line 1564) and saves changes from the limits editor (POST at line 1650). The POST
branch already exists and only needs the session guard and the CSRF header.

### 3.6 `/api/chart-settings` - chart axes

Same split pattern as the slider limits: GET (line 726) returns the chart axis
settings, POST (line 743) changes them with validation (max must exceed min, time
window and grid at least 1). The page saves chart settings at line 1782 of
`WebContentScript.h`. Same treatment as 3.5.

### 3.7 Static files and the fallback route

Three files have explicit routes: `/chart.min.js` (line 858), `/Logo.svg` (line 864)
and the user manual PDF (line 870). They are served by `handleFileRead()` (line 216),
which streams the file from LittleFS in 1 KB chunks so the limited RAM is never
overloaded.

**The fallback** (`onNotFound`, line 876) is the dangerous part: for any path without
an explicit route, it calls `handleFileRead(webServer.uri())` with the raw requested
path. Since `settings.json` and `default.json` are ordinary files on the flash, the
URLs `/settings.json` and `/default.json` serve them to anyone. That is finding 5.
The planned change adds a small check here that refuses these two files (minimal
P2.1). A complete path sanitizer is a separate measure.

---

## Part 4 - The Building Blocks on the ESP32

### 4.1 Cookies: the server's sticky note

A **cookie** is a small text the server hands to the browser with the instruction
"send this back to me with every future request." It is how a server remembers a
browser between requests, because HTTP itself is stateless: every request looks like
the first one.

The login response will set a session cookie with three safety flags:

- **HttpOnly**: the page's JavaScript cannot read the cookie. Even if script could be
  injected, the token stays out of reach.
- **SameSite=Lax**: the browser only attaches the cookie to requests that come from
  the same site. A malicious page on another site therefore cannot ride along on the
  cookie when it fires a POST at the valve. (Top-level GET navigations still carry it,
  which is exactly why mutation must not be GET.)
- **Path=/**: the cookie applies to the whole site.

### 4.2 The session token

The server does not store "the browser is logged in" per request. It generates one
random number at login time, called the **session token**, remembers it in RAM, and
puts it into the cookie. Every guarded request must present a cookie whose token
matches. The token comes from `esp_random()`, the ESP32 hardware random generator,
so it cannot be guessed.

A RAM-only token has one nice property: every reboot wipes it, so a restart always
ends all sessions.

### 4.3 PBKDF2 and why the hash is stored, not the password

The password itself is never stored on the flash and never sent back. At login, the
firmware derives a key with **PBKDF2-HMAC-SHA256** from the entered password and a
random per-password **salt** (computed with the mbedtls library that the ESP32 core
ships with) and compares it with the stored hash in `settings.json`. The salt plus
a high iteration count makes offline guessing slow; a plain SHA-256 hash would be
crackable at high speed. One-way means: the derived value cannot be turned back
into the password, so a stolen hash only allows time-consuming guessing, never
direct reading. The comparison itself uses a constant-time routine so that timing
does not leak how many characters matched.

Storing the hash in `settings.json` makes the password runtime-configurable, matching
the direction already decided for the AP password (measure P2.2). This is also why
the JSON files must stop being downloadable (Part 3.7): a leaked hash enables offline
password guessing.

There is no universal default password. A device whose `settings.json` contains no
hash is in the provisioning state: it only serves the setup page, and no control
endpoint answers until the operator sets a password (see 5.1). This is what makes
the design compliant with CRA Annex I, Part I, 2(b)/(d). ETSI EN 303 645 5.1-1 is
quoted as a technical benchmark only: it is not a harmonised act and creates no
presumption of conformity, neither under the CRA nor under the RED, which applies
until 2027-12-11.

### 4.4 Login rate limiting

To stop an attacker from trying passwords as fast as the ESP32 can hash, the login
handler counts failed attempts. After `MAX_LOGIN_ATTEMPTS` (5) failures within a
short window, logins are refused for `LOCKOUT_MS` (60 s). The counter lives in RAM,
next to the session state.

---

## Part 5 - How the Planned Authentication Works (Design)

### 5.1 The login flow

Before the first login there is one extra step: provisioning. On a fresh device
(or after a factory reset) the ESP32 has no stored password hash and serves the
setup page (`WebContentSetup.h`). The operator must set the password there; only
then does the login flow below become available.

```
Browser                        ESP32
───────                        ─────
GET /                          "Not logged in."
◀───── login page (HTML_LOGIN)

type password, submit
POST /login  (password) ─────▶ check hash, count attempts
                              generate session token (esp_random)
◀───── Set-Cookie: session=...; HttpOnly; SameSite=Lax; Path=/
      + JSON {success:true}

browser reloads
GET /  (with cookie) ────────▶ token valid → control page
```

The login page is a small PROGMEM chunk (`WebContentLogin.h`, added to the include
list in `WebContent.h`). `handleRoot()` streams it instead of the control page
whenever the request has no valid session. The control page itself stays exactly as
it is.

### 5.2 The session lifecycle

- **Birth**: successful `/login` sets the cookie and stores the token plus a
  timestamp in RAM.
- **Use**: every guarded request checks the cookie token (constant-time compare) and
  refreshes the timestamp, so 30 minutes of activity never expires mid-work.
- **Idle death**: if no guarded request arrives for `SESSION_TIMEOUT_MS`
  (30 minutes), the token stops being valid.
- **Logout**: `/logout` (POST) clears the cookie and the token immediately.
- **Reboot**: RAM is gone, so all sessions end.

### 5.3 Guarding the mutating endpoints

Each mutating handler starts with the same small check (`requireAuth()`):

```
if no cookie, or token mismatch, or expired:
    webServer.send(401, ...)          // "unauthorized"
    return
```

The page reacts to a 401 by showing the login page again. Applied to: `/set`,
`/resetPID`, `/api/slider-limits` POST, `/api/chart-settings` POST. Read-only
endpoints (`/values`, both GETs, static files) stay open in the operational state,
because reading the chart needs no secret and cannot change anything. In the
provisioning state they answer 403 as well (see 5.1); only static assets and
`/setup` are reachable.

### 5.4 CSRF protection: POST plus header check

Two changes together make Path B (Part 2.3) ineffective:

1. **POST only.** `/set` and `/resetPID` move to `webServer.on("/set", HTTP_POST, ...)`.
   No more GET mutation. The JavaScript switches from `fetch("/set?" + params)` to
   `fetch("/set", {method:"POST", body: params})` with a form-encoded body.
2. **Custom header.** Every mutating `fetch` adds the header
   `X-Requested-With: fetch`. A cross-site request from a malicious page cannot set
   this header (the browser would first send a permission preflight the ESP32 never
   answers), and it cannot ride on the session cookie either (SameSite=Lax). The
   server rejects any mutating request without the header (403).

This uses the browser's own rules as the defense line, which fits an ESP32 that
should not run a full token database.

### 5.5 Protecting the JSON configuration files

The `onNotFound` fallback (line 876) refuses `/settings.json` and `/default.json`
before `handleFileRead()` runs. The calibration values and the password hash are
therefore no longer downloadable. This is the minimal part of P2.1; a general path
sanitizer is separate work.

### 5.6 Login rate limiting

`handleLogin()` returns 429 ("too many requests") after 5 failed attempts and keeps
refusing for 60 seconds. Failed and successful logins are logged via the existing
`LOG_I(CAT_NETWORK, ...)` macros.

### 5.7 Which files change

| File | Change |
|---|---|
| `src/Constants.h` | new `AuthConfig` namespace: password length limits, session timeout, attempt limit, lockout time, CSRF header name/value (no default password) |
| `src/SettingsHandler.h/.cpp` | `auth_password_salt` + `auth_password_hash` + `auth_provisioned` fields, `checkPassword()` (PBKDF2), `setPassword()`, JSON load/save for the new fields |
| `src/WebHandlers.h/.cpp` | session state, `handleLogin()`, `handleLogout()`, `handleSetup()`, `requireAuth()`, route method changes, `handleRoot()` setup/login branches, provisioning gate, JSON blocking in `onNotFound` |
| `src/WebContentLogin.h` (new) + `src/WebContent.h` | login page chunk and its include |
| `src/WebContentSetup.h` (new) + `src/WebContent.h` | setup page chunk (first-boot provisioning) and its include |
| `src/WebContentScript.h` | POST conversion of `/set` and `/resetPID`, CSRF header on all mutating calls, login and setup form logic, 401 handling |
| `data/default.json`, `data/settings.json` | new `auth_password_salt` (empty), `auth_password_hash` (empty), `auth_provisioned` (false) fields |

---

## Part 6 - What Authentication Does Not Solve

Deliberately outside this measure, so expectations stay honest:

- **No encryption on the radio link.** The AP still uses the fixed, documented WPA2
  password. Anyone in radio range can still eavesdrop on the traffic and even read
  the session cookie. Real confidentiality needs measure P3.3 (a per-device password
  or an encrypted application protocol). Authentication keeps strangers from
  **commanding** the valve; it does not keep them from **listening**.
- **One shared account.** There is one password for everyone, not per-user accounts.
- **Read endpoints stay open.** `/values`, the GET APIs and the static files need no
  session.
- **Safety limits are separate measures.** Capping the sensor calibration to the
  physical sensor range (P1.3) and hard-capping the PID gains (P1.4) are independent
  of authentication and remain in the queue. Authentication only controls **who**
  may call `/set`; the validation limits control **what** `/set` may do.

---

## Appendix - Verification in One Picture

The complete verification procedure (curl commands, browser flows, stress test) is
documented in the companion plan: `../plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md`, section 5.
