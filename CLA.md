# TinkerRocket Contributor License Agreement

**Version 1.0**

Thanks for wanting to contribute. This agreement covers the legal side of that, once,
so neither of us has to think about it again.

> **Maintainer note:** this document is a draft derived from the Apache Software
> Foundation's Individual CLA, adapted to cover hardware design files as well as
> software. It has **not** been reviewed by a lawyer. If Tinkerbug Robotics intends to
> rely on the relicensing right in section 4 commercially, have counsel read it first.
> Delete this note once that has happened.

---

## In plain language, before the legal text

You keep the copyright in everything you contribute. You are not signing your work away.

What you grant is a **licence** — permission for Tinkerbug Robotics to use your
contribution, including the permission to release it under a different licence later.
That last part is the reason this document exists. TinkerRocket is GPL-3.0 for software
and CERN-OHL-S for hardware. If every contributor's work is locked to those licences,
the project can never change them without tracking down every person who ever sent a
patch. This agreement keeps that door open.

You can still do anything you like with your own contribution elsewhere. Granting a
licence is not exclusive.

If you would rather not sign, that is a completely reasonable position, and you are
still welcome to open issues, report bugs, and discuss designs.

---

## 1. Definitions

**"You"** means the individual who submits a Contribution, or the legal entity
authorising it. For an entity, the entity and all others that control it, are
controlled by it, or are under common control with it are treated as one.

**"Tinkerbug Robotics"**, **"we"**, and **"us"** mean the TinkerRocket project
maintainers.

**"Contribution"** means any work of authorship you intentionally submit to the project.
This expressly includes **both**:

- **software** — source code, firmware, application code, scripts, tests, build files,
  configuration and documentation; and
- **hardware design files** — schematics, PCB layouts, symbol and footprint libraries,
  fabrication outputs, mechanical drawings, and bills of materials.

"Submit" means any form of communication sent to us or our repositories — a pull
request, an issue attachment, an email, or a message on a project discussion channel —
except anything you conspicuously mark **"Not a Contribution"** in writing.

## 2. Copyright licence

You grant Tinkerbug Robotics and everyone who receives the project a perpetual,
worldwide, non-exclusive, royalty-free, irrevocable copyright licence to reproduce,
prepare derivative works of, publicly display, publicly perform, sublicense, and
distribute your Contribution and derivative works of it.

## 3. Patent licence

You grant Tinkerbug Robotics and everyone who receives the project a perpetual,
worldwide, non-exclusive, royalty-free, irrevocable (except as stated below) patent
licence to make, have made, use, offer to sell, sell, import, and otherwise transfer
your Contribution, whether alone or combined with the project.

This licence covers only those patent claims you can license that are necessarily
infringed by your Contribution alone, or by the combination of your Contribution with
the project it was submitted to.

If you begin patent litigation against anyone alleging that the project or a
Contribution in it constitutes direct or contributory patent infringement, every patent
licence you were granted under this agreement terminates as of the date that litigation
is filed.

## 4. Relicensing

You agree that Tinkerbug Robotics may license your Contribution under terms of its
choosing, including licences that differ from the project's current ones, and including
proprietary terms.

This is the clause that lets the project change direction — adopt a different open
source licence, or offer a commercial licence alongside the open one — without needing
to contact every past contributor. It does not restrict what you may do with your own
Contribution, and it does not withdraw the open source release of anything already
published.

## 5. What you are representing

By signing, you confirm:

1. **The work is yours.** Each Contribution is your original creation, or you have the
   right to submit it under this agreement.
2. **You have the authority.** If your employer has rights in work you create, you have
   permission to make the Contribution, your employer has waived those rights, or your
   employer has itself signed this agreement.
3. **You will flag anything that is not yours.** See section 6.

## 6. Third-party material

If your Contribution includes work you did not write — a vendored library, a reference
design, a footprint from a manufacturer, code adapted from elsewhere — you must say so
clearly, identify its source and licence, and note any restriction that applies.

Mark such material plainly in the Contribution itself, in the pull request description,
or with a note reading **"Submitted on behalf of a third party: [name and licence]"**.

TinkerRocket already vendors third-party components under their own terms; this section
exists to make sure new ones stay identifiable rather than being silently absorbed.

## 7. No obligations

You are not expected to provide support for your Contribution. You may, but you are not
required to.

Except for the representations in section 5, your Contribution is provided **"AS IS"**,
without warranties or conditions of any kind, express or implied, including any warranty
of merchantability, fitness for a particular purpose, or non-infringement.

Nothing here obliges us to merge, ship, or keep any Contribution.

## 8. If something changes

If any statement you made here becomes inaccurate — you learn you did not hold rights
you thought you held, for example — tell us at the contact address in the repository as
soon as you reasonably can.

---

## How to sign

State the following in the first pull request you open, in a comment on it:

```
I have read the TinkerRocket CLA and I agree to it.
Signed-off-by: Your Name <your.email@example.com>
```

Use a real name and an address that reaches you. One signature covers all your future
contributions; you do not repeat it per pull request.

**Safety-critical reminder.** TinkerRocket fires pyrotechnic devices and flies at
altitude. Contributions touching pyro, deployment, recovery, or flight-state logic get
proportionally more scrutiny, and may be asked for bench evidence before merge. That is
not distrust — it is that a mistake in those paths is not recoverable in flight.
