# Syncing recordings to Google Drive

`runs/video/<timestamp>` and `runs/audio/<timestamp>` are pushed to a personal
Google Drive folder by a background systemd timer, independent of ROS and of
whether the machine has internet access at recording time. A run only
uploads once its `metadata.json` reports `"state": "complete"`, so an
in-progress recording is never uploaded half-written. The timer re-runs every
five minutes and `rclone copy` only transfers files that differ from the
remote, so:

- offline during a recording -> the run just uploads on the first timer tick
  after connectivity returns.
- the timer or the machine was off when a run finished -> it is picked up
  the next time the timer fires, with no bookkeeping needed on our side.

This is one-time setup (roughly 20-30 minutes, mostly clicking through Google
Cloud Console); after that it runs with no human interaction indefinitely.

## 1. Install rclone (one-time, needs sudo)

```bash
sudo apt-get install -y rclone
```

## 2. Create your own Google Cloud OAuth client

Don't skip this and use rclone's default blank `client_id`/`client_secret` --
that uses a client shared by every rclone user worldwide, and it hits
Google's shared rate limit (`403 RATE_LIMIT_EXCEEDED`) quickly in practice.
A client scoped to this project avoids that entirely.

1. https://console.cloud.google.com/ -> create a new project (e.g.
   `dracoviloc-rclone`) and make sure it's selected.
2. https://console.cloud.google.com/apis/library/drive.googleapis.com ->
   **Enable**.
3. https://console.cloud.google.com/apis/credentials/consent -> **External**
   -> fill in app name / support email / developer contact -> **Save and
   Continue** through Scopes -> add your own Google account under **Test
   users** -> **Save and Continue**.
4. On the **Data Access** tab, **Add or Remove Scopes**, search `drive`, and
   check `.../auth/drive` (full access -- matches the `scope=drive` choice
   used below). Save.
5. On the **Branding** tab, fill in the two fields that block publishing
   later:
   - **Application home page**: your repo's URL, e.g.
     `https://github.com/<you>/DracoViLoc`
   - **Application privacy policy link**: a reachable URL with real content.
     This repo has one at [`PRIVACY.md`](../PRIVACY.md) -- use
     `https://github.com/<you>/DracoViLoc/blob/main/PRIVACY.md`.
6. https://console.cloud.google.com/apis/credentials -> **Create
   Credentials** -> **OAuth client ID** -> Application type **Desktop app**
   -> name it anything -> **Create**. Note the **Client ID** and **Client
   secret** shown (or re-open the client later to view the secret again if
   you close the popup).

Save these two values in `scripts/drive_oauth_client.env` (gitignored --
never commit this file):

```bash
GOOGLE_DRIVE_CLIENT_ID=...
GOOGLE_DRIVE_CLIENT_SECRET=...
```

This is just a local reference so you don't have to re-open Cloud Console to
look them up if you ever need to reconfigure rclone (new machine, etc.).

## 3. Authorize rclone with that client (one-time, needs your Google login)

This step cannot be automated or done on your behalf -- it is your personal
account's OAuth consent.

```bash
rclone config
# n) New remote
# name>                 pick anything, e.g. gdrive (just a local label)
# Storage>              drive
# client_id>            <from scripts/drive_oauth_client.env>
# client_secret>        <from scripts/drive_oauth_client.env>
# scope>                1   (full access)
# root_folder_id>       (leave blank)
# service_account_file> (leave blank)
# Edit advanced config? n
# Use auto config?      y    <- opens a browser to log in and consent
# Configure as a Shared Drive? n
```

If this machine is headless (no browser, e.g. SSH-only), answer
`Use auto config? n` instead -- it prints a `rclone authorize "drive"`
command to run on any machine that *does* have a browser, which prints a
token to paste back into the prompt here.

Verify it worked (quote the name if it has spaces):

```bash
rclone listremotes
rclone lsd "<remote>:"    # should list your actual Drive folders
```

If you instead get `403 ... RATE_LIMIT_EXCEEDED`, you likely left
`client_id`/`client_secret` blank -- edit the remote (`rclone config` -> `e`)
and fill them in from step 2.

## 4. Publish the app (one-time) -- avoids re-authenticating every week

While the OAuth consent screen is in **Testing** status, Google force-expires
refresh tokens after 7 days, which would silently break the unattended sync.
Fix this once:

1. https://console.cloud.google.com/apis/credentials/consent -> **Audience**
   tab -> **Publish App**.
   - If the button is locked with a message about required branding fields,
     go fill in the Homepage/Privacy Policy URLs from step 2.5 above, then
     retry.
2. Mint a fresh token under the new Production status (the one you already
   have was issued under Testing and keeps that 7-day limit):

   ```bash
   rclone config
   # e) edit the remote -> press Enter through every prompt to keep values
   # y) accept the summary
   # Already have a token - refresh?  y
   # Use auto config?                 y   <- browser again, log in once more
   # Configure as a Shared Drive?     n
   ```

After this, the refresh token has no fixed expiry -- only explicit
revocation, a Google account security event, or ~6 months of total
inactivity would invalidate it, and the 5-minute sync timer means the last
one never happens.

## 5. Install the background timer

```bash
~/DracoViLoc/scripts/install_drive_sync.sh
```

This copies the unit files to `~/.config/systemd/user/`, enables the timer,
and warns if rclone or the remote aren't ready yet (the timer is still safe
to enable early -- it just logs failures until steps 2-4 are done).

Lingering (`loginctl enable-linger <user>`) is required so the timer runs
even when you are not logged in; the install script checks and tells you if
it's missing.

## Configuration

Create `~/.config/dracoviloc/drive_sync.env` to point at your actual remote
name and destination folder (defaults are placeholders and won't match a
freshly configured remote):

```bash
DRIVE_SYNC_REMOTE_NAME=gdrive
DRIVE_SYNC_REMOTE_PATH=DracoViLoc/runs
```

## Checking on it

```bash
systemctl --user status sync-runs-to-drive.timer
systemctl --user list-timers sync-runs-to-drive.timer
tail -f ~/DracoViLoc/runs/.drive_sync.log
```

Trigger a sync immediately instead of waiting for the next tick:

```bash
systemctl --user start sync-runs-to-drive.service
```
