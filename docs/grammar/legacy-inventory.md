# Legacy inbox migration inventory

The quarantined source at
`INBOX/taos_legacy_preservation_intial_work_maybe_faulty/` is treated as
research input, not as a second canonical implementation. The reproducible
file-level inventory and SHA-256 record is generated at
`metadata/legacy_inbox_manifest.json` by:

```bash
python tools/dev.py legacy-audit
```

Migration completion is checked separately:

```bash
python tools/dev.py legacy-close-check
```

That command is expected to fail while any provisional file disposition or
grammar candidate remains. It becomes the completion signal only after every
artifact has been adapted, preserved canonically, archived as research, or
discarded, and every grammar entry has an accepted, archived, or rejected
status.

Each file receives one disposition:

- `archive-research`: legacy evidence, candidate grammar, schemas, examples,
  tests, source, and notes retained as research but excluded from canonical
  runtime claims.
- `discard-generated`: caches, wheels, release artifacts, and local metadata.

Migration is complete only when every non-generated artifact has either been
adapted into the canonical repository with tests and provenance, or has been
deliberately retained as documented research material. The legacy package,
provisional grammar claims, and synthetic examples must not silently become
verified TAOS syntax or simulation behavior.
