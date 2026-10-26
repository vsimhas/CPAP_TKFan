#!/usr/bin/env python3
"""Adherence metrics from a SUMMARY.CSV (checklist #65), computed the way the
firmware's adherence.c defines them, to cross-check `therapy adherence`.

Definitions (adherence.h): a session counts toward the night it STARTED in,
split at noon (a start before 12:00 belongs to the previous date); usage is
mask_on_s; records with rtc_valid != 1 are undated and left out. Window =
the last 30 nights ending tonight, never before first use; the reimbursement
test is the best 30-night window inside the first 90 nights from first use,
met at >= 21 nights with >= 4 h.

usage: python adherence_check.py SUMMARY.CSV [--now "YYYY-MM-DD HH:MM:SS"]
"""
import argparse
import csv
import datetime as dt

WINDOW, THRESH_S, INITIAL, NEEDED = 30, 4 * 3600, 90, 21


def night(t):
    d = t.date()
    return d - dt.timedelta(days=1) if t.hour < 12 else d


def compute(path, now=None):
    usage, sessions, undated, total = {}, 0, 0, 0
    with open(path, newline="", encoding="ascii") as fh:
        rows = [r for r in csv.reader(l for l in fh if not l.startswith("#"))]
    hdr = None
    for r in rows:
        if r and r[0] == "schema":
            hdr = {k: i for i, k in enumerate(r)}
            continue
        if hdr is None or not r:
            continue
        sessions += 1
        try:
            if int(r[hdr["rtc_valid"]]) != 1:
                raise ValueError
            t = dt.datetime.strptime(r[hdr["start"]], "%Y-%m-%d %H:%M:%S")
        except (ValueError, IndexError):
            undated += 1
            continue
        s = max(0, int(r[hdr["mask_on_s"]] or 0))
        total += s
        n = night(t)
        usage[n] = usage.get(n, 0) + s
    if not usage:
        return dict(sessions=sessions, undated=undated)
    first = min(usage)
    today = night(now) if now else max(usage)
    frm = max(first, today - dt.timedelta(days=WINDOW - 1))
    days = [frm + dt.timedelta(days=i) for i in range((today - frm).days + 1)]
    win = [usage.get(d, 0) for d in days]
    since = (today - first).days + 1
    span = min(INITIAL, max(0, since))
    init = [usage.get(first + dt.timedelta(days=i), 0) for i in range(span)]
    if span >= WINDOW:
        best = max(sum(1 for s in init[k:k + WINDOW] if s >= THRESH_S)
                   for k in range(span - WINDOW + 1))
    else:
        best = sum(1 for s in init if s >= THRESH_S)
    n = len(days)
    used = sum(1 for s in win if s > 0)
    ge4 = sum(1 for s in win if s >= THRESH_S)
    return dict(night=today, days=n, used=used, ge4h=ge4,
                pct4h=(100 * ge4 + n // 2) // n, avg_min=sum(win) // 60 // n,
                avg_used_min=(sum(win) // 60 // used) if used else 0,
                first=first, since=since, sessions=sessions, undated=undated,
                total_h=total // 3600, best30=best, partial=span < WINDOW,
                met=int(best >= NEEDED))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("summary")
    ap.add_argument("--now", help='"YYYY-MM-DD HH:MM:SS" (default: last night in the file)')
    a = ap.parse_args()
    now = dt.datetime.strptime(a.now, "%Y-%m-%d %H:%M:%S") if a.now else None
    r = compute(a.summary, now)
    if "night" not in r:
        print("no dated sessions:", r)
    else:
        print("# ADH night=%s days=%d used=%d ge4h=%d pct4h=%d avg_min=%d avg_used_min=%d "
              "first=%s since=%d sessions=%d undated=%d total_h=%d best30=%d%s met=%d"
              % (r["night"], r["days"], r["used"], r["ge4h"], r["pct4h"], r["avg_min"],
                 r["avg_used_min"], r["first"], r["since"], r["sessions"], r["undated"],
                 r["total_h"], r["best30"], "/partial" if r["partial"] else "", r["met"]))
