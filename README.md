# Altitude CanSat 2026/2027

![Altitude team logo](assets/images/Team%20Altitude%20Logo.png)

Altitude is a student-led CanSat team from Haberdashers' Boys' School in Hertfordshire, competing in the 2026/2027 season. The team is building a miniature satellite payload that combines embedded programming, CAD, 3D printing, telemetry, sensor testing, and clear engineering documentation.

## Team

- Eric - Code | CAD
- Lewis - Team Leader
- Sebastian - Build
- Kamran - Notebook | CAD

## The CanSat

The primary mission measures air temperature and pressure during descent and transmits both once every second. The secondary mission records UVA and UVB at different altitudes, using four selected SparkFun Mini AS7331 boards. Four body-mounted ASA Aero arms at 90-degree intervals point the optical faces outward and 30 degrees upward. The canopy still shades zenith; angular response and final canopy geometry need characterization.

Python with MicroPython on ESP32-S3 remains the development platform. The simplified architecture proposes one main flight controller. The initial custom structure is entirely 3D printed in ASA Aero and TPU85A, with a reusable fabric parachute for passive recovery. The arm study recommends 45 mm free span × 12 mm width × 12 mm thickness with captured roots and positive locks. Its reported stress and movement are analytical estimates; no FEA/CFD run completed because SimScale sign-in failed. Final deployment mechanism, packaging, wiring, calibration and flight qualification remain to be completed.

The Bambu Lab X2D printer, AMS hardware and filament dryers support fabrication and material conditioning. Each material uses the appropriate drying settings and feed path.

The [current PDR and UV design](https://github.com/ericxyuan/altitude-cansat/blob/main/docs/UV_MISSION.md), [Onshape demonstration model](https://cad.onshape.com/documents/76123ac2946988d574b71b35/w/4e6885d706dac03531484551/e/0391f996e69f00881814257a) and [Lucid system diagram](https://lucid.app/lucidchart/16d928bb-fa47-4260-83cb-e005f5ee5b93/edit) describe the current mission.

## Website

The site is a static HTML, CSS, and JavaScript website in the `website/` folder, with shared assets in `assets/`. Root-level HTML files redirect into the website folder so the pages work cleanly on GitHub Pages.

The main engineering repository for flight code, CAD, telemetry work, build notes, and test data is [ericxyuan/altitude-cansat](https://github.com/ericxyuan/altitude-cansat).

Pages included:

- Home
- Team
- CanSat
- About Altitude
- About CanSat

## Interactive yaw simulator

The [Simulations page](website/simulations.html) adds a client-side reaction-wheel yaw model with live hardware inputs, multiple comparison trajectories, torque/speed limits and an independent Python reference. It preserves the existing website and uses self-hosted Chart.js 4.5.1 under its MIT licence. Read the [model assumptions and numerical validation](docs/SIMULATION_MODEL.md) before interpreting the predictions as hardware performance.

From this website repository root, run the standalone checks:

```sh
node --test tests/simulation-physics.test.mjs
python tests/simulation_reference.py
```

The Node suite uses Python 3 for cross-validation; set `PYTHON` if its executable has a different name. These checks require no package installation. This static site has no configured lint, typecheck or build command. Numerical tests do not establish physical or flight readiness.

## GitHub Pages, A Records, And HTTPS

The Pages repository is [ericxyuan/ericxyuan.github.io](https://github.com/ericxyuan/ericxyuan.github.io), matching the current GitHub username. The default Pages address is `https://ericxyuan.github.io/`; the public custom domain below remains the main website address.

The custom domain for the site is:

```txt
altitude.linkpc.net
```

The root `CNAME` file in this repository is intentional. It is GitHub Pages metadata that stores the custom domain name for the published site. It does **not** mean the DNS control panel has to use a DNS CNAME record.

If `altitude.linkpc.net` is being managed as the apex/root of its own DNS zone, use GitHub Pages A records:

```txt
A  @  185.199.108.153
A  @  185.199.109.153
A  @  185.199.110.153
A  @  185.199.111.153
```

If the DNS zone is `linkpc.net` instead, use the host/name `altitude` instead of `@` for those same A records.

Optional IPv6 records:

```txt
AAAA  @  2606:50c0:8000::153
AAAA  @  2606:50c0:8001::153
AAAA  @  2606:50c0:8002::153
AAAA  @  2606:50c0:8003::153
```

After the A records resolve correctly, GitHub Pages can issue an HTTPS certificate. In the repository settings, go to **Settings > Pages**, confirm the custom domain is `altitude.linkpc.net`, wait for the certificate check to finish, then enable **Enforce HTTPS**.

If the DNS zone has CAA records, make sure Let's Encrypt is allowed so GitHub Pages can issue the certificate:

```txt
CAA  @  issue  "letsencrypt.org"
```

GitHub Pages documentation:

- [Managing a custom domain for GitHub Pages](https://docs.github.com/en/pages/configuring-a-custom-domain-for-your-github-pages-site/managing-a-custom-domain-for-your-github-pages-site)
- [Securing your GitHub Pages site with HTTPS](https://docs.github.com/en/pages/getting-started-with-github-pages/securing-your-github-pages-site-with-https)
