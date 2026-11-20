# AMR Mastery: Certificate of Completion

> **Status:** the submission form opens soon. Its link will be added here and in the pinned comment of Lesson 40. You can start your submission repo today: everything below already applies.

AMR Mastery is a **free course** on the [MechaMind Labs YouTube channel](https://www.youtube.com/playlist?list=PLae-2ikc0DUY): 40 lessons, 22 assignments, and every resource free. You get the slides, PDFs, command lists, CAD and STL files, meshes, URDF, circuit diagrams, assignment briefs and all the code in this repository. Learn at your own pace.

If you complete the assignments and the final project and submit your work, it is **reviewed by hand**. If it passes, you receive an official **MechaMind Labs Certificate of Completion**. The certificate is for practical work you can show, not for watching the videos.

## Two certificates

| | **Simulation Track** | **Hardware Track** |
|---|---|---|
| Lessons | 1–33 and 40 (34 lessons) | All 40 lessons |
| Assignments | AS01–AS17 + AS22 (18) | AS01–AS22 (22) |
| Final project | In simulation | In simulation **and** on a physical robot |
| Hardware needed | None | Your own robot (MechaBot or your own design) |

You can start with the Simulation Track and upgrade later. Add the hardware work to the same repo and submit again; the Simulation certificate stays valid.

## What you need to complete

| Requirement | Details |
|---|---|
| Assignments | Simulation Track: AS01–AS17 and AS22. Hardware Track: also AS18–AS21. Each in its own folder with code, a short README and evidence (GIF, screenshot or video link). Up to 2 of AS01–AS17 may be submitted as "attempted", with an honest write-up of what did not work |
| Final project (capstone) | Your MechaBot in **your own Gazebo world**: map it, save the map, localize, patrol at least 3 waypoints, then return and dock on **your own docking station design** (AS07). Make at least one visible change to the robot so it is yours |
| Demo video | 3–6 min, unlisted on YouTube, one continuous run of the mission. Start by saying your name and the date (face or voice), and end by explaining one thing you changed or debugged |
| Report | `docs/report.md`, 2–4 pages: what you built, architecture diagram, your changes, hardest problems, results and limits, next steps |
| Hardware (Hardware Track only) | AS18–AS21 on a physical robot + a 2–5 min hardware demo video of the same mission, with you visible or audible at least once |

## How to submit

1. **Start your repo** from the template in [`certification/submission-template/`](certification/submission-template/). Commit as you go: a history built over weeks is part of the evidence.
2. **Finish** the assignments and the capstone; fill in the READMEs and the report.
3. **Tag** the version you want reviewed: `git tag v1.0-submission && git push --tags`.
4. **Upload** your demo video(s) to YouTube as *Unlisted*.
5. **Submit** the form (link coming soon). It asks for your repo URL and tag, your video links, your name as it should appear on the certificate, and three short questions about your own system.

You get an automatic email within minutes. If something is missing (repo not public, tag not found, folders missing), the email tells you exactly what to fix.

## How your work is evaluated

| Section | Points |
|---|---|
| Assignments of your track | 35 |
| Capstone works end to end | 25 |
| ROS 2 package and code quality | 10 |
| Repository structure and Git history | 5 |
| Documentation and report | 10 |
| Demo video and understanding | 15 |

- **Pass:** 70 or more **and** all of these are true:
  - the assignments are complete
  - the capstone runs in one take
  - the work is clearly your own
  - you can explain it
  - the report is there
  - the declaration is honest
- **Pass with distinction:** 90 or more.
- If it does not pass yet, you get your scores and specific notes, and you can resubmit up to two times.

## Timeline

You get the acknowledgement and the automatic pre-check within minutes. Reviews are done weekly, with a target of 14 days. When your work passes, you get the certificate as a PDF by email.

## Verifying a certificate

Every certificate has an ID (for example `MML-AMR-SIM-2026-0001` or `MML-AMR-HW-2026-0001`) and a QR code. Both lead to a public verify page showing the name, result and a link to the project. Anyone, including an employer, can check it in one scan.

## Rules

1. Your work must be your own. Using the course code is expected; credit anything you reused in your README.
2. The capstone video must show you (face or voice) and one continuous run of the mission.
3. Keep your repo public at the submitted tag until your certificate is issued.
4. Up to two resubmissions; after that, wait 30 days.
5. A certificate based on copied work is revoked and shown as revoked on the verify page.

## FAQ

- **Do I need hardware?** No. The Simulation Track certificate needs no hardware.
- **Is there a deadline?** No. Submit when your work is ready.
- **I was in a 2026 live batch.** Certificates already issued stay valid.
- **Questions?** Ask in the comments of the relevant lesson on YouTube.
