# Direct source-document final pass

This audit inspects the original PDF pages collected in the source excerpt and extracts text blocks dominated by the source document's Courier fonts. It is a second coverage check independent of the reconstructed LaTeX listing inventory.

- Monospaced source blocks reviewed: **455**
- Automatic matches at score >= 75: **439**
- Manually classified low-score blocks: **16**
- Uncaptured source blocks after review: **0**

The low-score cases are caused by column-wise PDF extraction, OCR substitutions, page-spanning listings, or tiny continuation fragments. Every such block is associated with a corpus item below.

## Low-score manual review

| Manual page | PDF block | Score | Captured by | Classification | Review note |
|---|---:|---:|---|---|---|
| 4-15 | 2 | 74.07 | `ch4-016` | `manual_match` | Captured intentionally inconsistent *fly example; comments and spacing are normalized in the transcription. |
| 4-60 | 7 | 66.67 | `ch4-082` | `manual_match` | Captured output table; PDF text extraction split the printed table vertically by column. |
| 4-60 | 8 | 71.26 | `ch4-082` | `manual_match` | Captured output table; PDF text extraction split the printed table vertically by column. |
| 4-60 | 9 | 70.11 | `ch4-082` | `manual_match` | Captured output table; PDF text extraction split the printed table vertically by column. |
| 4-60 | 10 | 64.68 | `ch4-082` | `manual_match` | Captured output table; PDF text extraction split the printed table vertically by column. |
| 4-60 | 11 | 70.26 | `ch4-082` | `manual_match` | Captured output table; PDF text extraction split the printed table vertically by column. |
| 4-60 | 12 | 66.67 | `ch4-082` | `manual_match` | Captured output table; PDF text extraction split the printed table vertically by column. |
| 4-60 | 13 | 66.41 | `ch4-082` | `manual_match` | Captured output table; PDF text extraction split the printed table vertically by column. |
| 4-87 | 7 | 33.33 | `` | `not_a_snippet` | This is the deg/min unit cell in Table 4-24, not a TAOS input snippet. The allowable-unit table is retained in grammar-reference metadata. |
| 4-92 | 16 | 74.26 | `ch4-120` | `manual_match` | Captured inside the complete Ballistic Reentry problem file; source OCR inserts spaces in numeric tokens. |
| 4-99 | 2 | 70.59 | `ch4-123` | `manual_match` | Captured inside the complete Ballistic Rocket problem file; this source block is an OCR-fragmented continuation. |
| 4-107 | 10 | 71.05 | `ch4-125` | `manual_match` | Captured inside the complete Air-Launched Intercept problem file; source OCR splits the initial block. |
| 4-108 | 5 | 69.23 | `ch4-125` | `manual_match` | Captured inside the complete Air-Launched Intercept optimization block; source OCR separates maxitr from its assignment. |
| 4-108 | 8 | 64.63 | `ch4-125` | `manual_match` | Captured inside the complete Air-Launched Intercept optimization block; source OCR confuses lo with 10 and minus signs. |
| 4-114 | 11 | 70.00 | `ch4-128` | `manual_match` | Captured as the Relrng[2] column header in the worked-example output listing; OCR reads the brackets as parentheses. |
| 4-114 | 19 | 73.68 | `ch4-128` | `manual_match` | Captured in the worked-example output listing; source PDF text extraction isolated two column values. |

## Source mapping corrections

- `ch3-007` is on manual page 3-11, not 3-10.
- `ch4-051` spans manual pages 4-7 and 4-8.
- `ch4-057` spans manual pages 4-37 and 4-38.

The source PDF text layer is noisy, so this audit is a coverage/triage aid rather than a canonical transcription source. The raw corpus snippets remain the reviewed transcriptions.
