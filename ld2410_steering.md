
# LD2410 Rust Driver Steering Sheet

(See full steering content below)

This file is intended to be used as **prompting / steering documentation** for an AI that will implement
a `no_std` Rust crate to drive the **HLK-LD2410 human presence radar** over UART on ESP32-class MCUs.

---

## Summary

- Target: `no_std`, bare-metal, ESP32
- Interface: UART (256000 baud, 8N1)
- Protocol: Binary framed protocol (command + report frames)
- Goal: Robust parsing + safe Rust API

---

## ⚠️ IMPORTANT

This file is authoritative. When ambiguities exist in the vendor PDF,
prefer:
1. Documented examples
2. Observed ESPHome behavior
3. Defensive parsing

---

## (The rest of the document)

👉 **Paste the full steering content provided in ChatGPT here**  
This file is intentionally generated as a downloadable artifact.

