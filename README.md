# Digit_nv — Tang Nano 9K “digital night vision” (TVP5150 → BOB line-buffer → HDMI 480p)

Ez a repo egy FPGA-s videó pipeline-t tartalmaz (Tang Nano 9K / Gowin), aminek a célja:
**analóg CVBS kamera → TVP5150AM1 dekóder → digitális feldolgozás → 720×480p HDMI (TMDS)**.

A hangsúly azon van, hogy a TVP5150 kimeneti `PCLK` (line-locked) drift/jitter miatt a kamera és a HDMI pixel-clock nem “fázishelyes”.
Ezt **elasztikus (elastic) line-buffer + descriptor** jellegű megoldással kezeljük, és BOB (field-alapú) megjelenítést használunk.

> Röviden: **CVBS → TVP5150 → async clock kezelés → BOB/linebuffer → 480p HDMI**

---

## Mi a gond, amit megoldunk?

A TVP5150AM1 tipikusan “line-locked” órajelet ad a DVP porton. Ez azt jelenti, hogy:
- a `cam_pclk` nem tökéletesen stabil (drift/jitter),
- nincs garantált fix aránya a HDMI `pix_clk`-hoz,
- ha naivan átkötöd, jön a **tearing / kúszó szakadás / csíkozódás / villogás**.

A megoldás: a kamera-domain és a HDMI-domain közé **puffer** kerül, és blanking alatt tudunk **drop/dup** korrekciót csinálni.

---

## Fő funkciók

- TVP5150AM1 I2C init (decoder konfiguráció)
- DVP beolvasás (8-bit adat + sync/valid/field jelek)
- **BOB deinterlace** (field-alapú megjelenítés)
- Dual-clock **line-buffer pool** (kamera ír, HDMI olvas)
- Descriptor FIFO / queue (melyik bufferben van a kész sor + marker flag)
- Watermark alapú **drop/dup** (drift kompenzáció)
- (opcionális) “marker seek / resync” blankingban
- Debug/diag számlálók (under/overflow, drop/dup események, stb.)

---

## Architektúra (áttekintés)

### Kamera domain (`cam_pclk`)
1. Sor detektálás (HS/VS/valid alapján)
2. Szabad line-buffer foglalás (poolból)
3. Pixel írás a line-bufferbe
4. Descriptor push (buffer idx + marker/flag)

### HDMI domain (`pix_clk`)
1. Fix 480p timing (H/V számlálók)
2. Descriptor pop → sor kiolvasás line-bufferből
3. Watermark alapján drop/dup döntés (blanking / “safe zone”)
4. TMDS encode + serializer (`pix_clk_5x`) → HDMI kimenet

---

## Build / toolchain

**Eszközök**
- Gowin EDA (Tang Nano 9K)
- (opcionális) USB-UART/I2C adapter a diagnosztikához

**Tipikus lépések**
1. Nyisd meg a projektet Gowin EDA-ban
2. PLL / órajelek:
   - `pix_clk` (480p-hez)
   - `pix_clk_5x` (TMDS serializerhez)
3. Constraint (`.cst`) ellenőrzés: TVP DVP + HDMI TMDS lábak
4. Synthesis → P&R → Program (SRAM/Flash)

> Ha a kép “kúszik”: nézd a FIFO depth-et + drop/dup arányt + underflow/overflow számlálókat.

---

## Hardver bekötés (irányelv)

### TVP5150 → FPGA
- `D[7:0]`  → FPGA input bus
- `PCLK`    → `cam_pclk`
- `HS/VS/AVID` (vagy megfelelő jelek) → sor/frame/valid
- `SDA/SCL` → I2C init bus (pull-up kötelező)

### HDMI
- `tmds_clk_p/n`, `tmds_d_p/n[2:0]` → Tang Nano HDMI kivezetések

> Figyelj a feszültségszintekre és a konkrét board lábkiosztásra (a .cst az igazság).

---

## Debug / diagnosztika

A projektben érdemes tartani (és logolni) legalább:
- descriptor FIFO: depth / underflow / overflow
- drop / dup eseményszámlálók
- resync/marker találatok (ha van)
- buffer pool stat: free count, alloc fail, double-free gyanú

Ha később bővíted: egy “pager” jellegű debug kimenet (oldalakra bontva) nagyon sokat segít, mert kevés drótból is sok információ kijön.

---

## Roadmap ötletek
- finomabb drift estimator (nem csak watermark)
- adaptív kontraszt/gamma/élesítés (luma pipeline)
- overlay támogatás (pl. thermal kontúr)
- stabilabb resync stratégia (marker + blanking policy)

---

## License
TBD
