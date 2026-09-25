# Handoff: CoMaps "smart lane assist" Android build (Germany)

## Goal
Custom build of **CoMaps** (offline OSM navigation, fork of Organic Maps) as an Android APK for driving in **Germany**:
1. Fully offline (maps copied from PC now and then).
2. **Smart lane assist**: when several lanes are valid, recommend only ONE, the lane on the side of the next maneuver, so the driver doesn't have to cross lanes later.
   - City: e.g. going straight at a 3-lane junction with a left turn coming up → show only the leftmost straight lane.
   - Autobahn: with no maneuver ahead, recommend the **rightmost** lane (German Rechtsfahrgebot, StVO §7). Move left only when a left fork/exit is ahead.
3. Show speed limit signs. **Stock CoMaps already does this** (OSM maxspeed); no change needed.

## Repo
- Fork: https://github.com/DX5536/comaps_TestLane (upstream: github.com/comaps/comaps)
- Local path (Windows): `C:\Users\Hai Lam\Desktop\Personal_GitKraken_Repo\comaps_TestLane`
- User uses the GitKraken GUI (its terminal is PowerShell) and Android Studio. The lane changes are committed and pushed to `main`.

## Code change (done, compiles; unit tests written but not run)
Files:
- `libs/routing/lanes/lanes_recommendation.hpp`: added `struct LaneChangeSettings` (`m_cityLookaheadMeters=1000`, `m_highwayLookaheadMeters=3000`, `m_highwayPreferRight=true`, `m_enabled=true`), `GetLaneChangeSettings()` and `MinimizeLaneChanges(...)`.
- `libs/routing/lanes/lanes_recommendation.cpp`: `SelectRecommendedLanes()` now calls `MinimizeLaneChanges()` at the end.
  - Walks the route segments **backwards** and remembers the side (left/right) of the next turn. Straight junctions (`IsStayOnRoad`) pass that side through; roundabouts and the destination reset it.
  - When a segment has ≥2 recommended lanes and the next maneuver is within the look-ahead distance, it keeps only the leftmost or rightmost recommended lane.
  - Motorway/trunk (not a link) with nothing pending → keeps the rightmost lane (the leftmost if `m_highwayPreferRight=false`).
- `libs/routing/routing_tests/lanes/lanes_recommendation_tests.cpp`: 2 new tests (`MinimizeLaneChanges_PrepareForLeftTurn`, `..._PrepareForRightTurn`).
- Lane hints only appear where OSM has `turn:lanes` tags.

## Build status: NOT yet successful
Target: Android Studio, variant **`fdroidDebug`**, ABI `arm64-v8a` (Samsung phone). Build with Build → Generate App Bundles or APKs → Generate APKs. Output: `android\app\build\outputs\apk\fdroid\debug\`. Install by copying the APK to the phone.

Already fixed:
- Gradle JDK set to **JDK 21**. Java 27 was too new for Gradle 8.14; the project targets Java 21.
- Submodules re-downloaded after an interrupted clone. jansson, pugixml and utfcpp were force-checked-out (`git submodule update --init --force`). Unclear whether protobuf/protobuf, just_gtfs, tools/kothic and tools/osmctools finished → re-run `git submodule update --init --force --recursive`.

Remaining errors (last build log):
1. `values-en/strings.xml`, `values-zh-rHK`, `values-zh-rMO` (app + sdk): "Content is not allowed in prolog". Cause: the repo has **110 git symlinks** (mode 120000) checked out as plain text files on Windows.
   Fix: Windows **Developer Mode ON** + `git config core.symlinks true` + delete and re-checkout every symlink. Two scripts for this are in the repo root: **`fix_symlinks.bat`** (double-click) and `fix_symlinks.ps1`. They also restore #2. Check with `(Get-Item android\app\src\main\res\values-en\strings.xml).LinkType` → should print `SymbolicLink`.
2. `3party/icu` "does not contain a CMakeLists.txt": the file is tracked in git but missing on disk → `git checkout -- 3party/icu/CMakeLists.txt`.
3. `3party/glaze` needs **CMake ≥ 3.31**; Android Studio uses 3.22.1. `android/sdk/build.gradle.kts` sets cmake `version = "3.22.1+"`. Fix: SDK Manager → SDK Tools → Show Package Details → install CMake 3.31+ (or 4.x). If Gradle still picks 3.22.1, set the exact version in `build.gradle.kts` or `cmake.dir` in `android/local.properties`.

After fixing: Build → Clean Project → Generate APKs.

Notes:
- `fix_symlinks.*` and `fix_symlinks_log.txt` are temporary; don't commit them.
- `./configure.sh` (Git Bash, with `SKIP_GENERATE_SERBIAN_LATIN_STRINGS=1`, needs Python 3 and `jq`) should have been run once; confirm it completed.
- Maps: after installing, download Germany (per state) in the app, or copy `.mwm` files to the phone.
