# Altitude CanSat 2026/2027

![Altitude team logo](assets/images/Team%20Altitude%20Logo.png)

Altitude is a student-led CanSat team from Haberdashers' Boys' School in Hertfordshire, competing in the 2026/2027 season. The team is building a miniature satellite payload that combines embedded programming, CAD, 3D printing, telemetry, sensor testing, and clear engineering documentation.

## Team

- Eric - Code | CAD
- Lewis - Team Leader
- Sebastian - Build
- Kamran - Notebook | CAD

## The CanSat

The Altitude CanSat uses Python flight software with MicroPython on the selected ESP32-S3 platform. Two physical controllers separate main guidance from the guardian that owns the servo outputs. The DevKitC-1 N8R8 is the recommended bench model (8 MB flash and 8 MB PSRAM); final flight packaging and power remain to be checked.

The fabrication workflow uses the Bambu Lab X2D printer with Automatic Material System hardware. The printer is used to manufacture the CanSat structure, internal fixtures, test pieces, and CAD iteration parts. The AMS 2 Pro and two AMS HT filament dryers support the fabrication process by managing engineering filaments and keeping materials dry for consistent print quality.

Current materials include:

- ASA Aero filament
- TPU 90A
- PETG CF
- PLA HS

The primary mission records and transmits atmospheric temperature and pressure. The secondary mission investigates GPS-guided descent with rotation-aware airbrake control. Hardware has not yet been purchased; host software tests do not establish flight readiness.

## Website

The site is a static HTML, CSS, and JavaScript website in the `website/` folder, with shared assets in `assets/`. Root-level HTML files redirect into the website folder so the pages work cleanly on GitHub Pages.

The main engineering repository for flight code, CAD, telemetry work, build notes, and test data is [ericxyn/Altitude-HABS-CanSat-2026-2027](https://github.com/ericxyn/Altitude-HABS-CanSat-2026-2027).

Pages included:

- Home
- Team
- CanSat
- About Altitude
- About CanSat

## GitHub Pages, A Records, And HTTPS

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
