# Submission template

Use this to start the repo you will submit for the [AMR Mastery Certificate of Completion](../../CERTIFICATION.md).

```bash
mkdir amr-mastery-<your-github-username> && cd $_ && git init
bash <path-to>/AMR-Mastery/certification/submission-template/setup_submission_repo.sh
git add . && git commit -m "Start AMR Mastery submission repo"
```

The script creates `assignments/AS01…AS22` (each with a README to fill in), `cad/`, `capstone/`, `docs/report.md`, `media/`, `mechabot_ws/src/`, a `.gitignore` and your `README.md`. Push it to a **public** GitHub repo and keep committing as you work.

| File | Becomes |
|---|---|
| `README_TEMPLATE.md` | your repo's `README.md` |
| `ASSIGNMENT_TEMPLATE.md` | `assignments/ASxx_*/README.md` |
| `CAPSTONE_TEMPLATE.md` | `capstone/README.md` |
| `REPORT_TEMPLATE.md` | `docs/report.md` |
| `gitignore.txt` | `.gitignore` |
