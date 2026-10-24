// ============================================================================
//  Minimal diagnostic sketch for verifying LSE (32.768 kHz) oscillation
//  Polls the LSERDY flag in the RCC_BDCR register directly, without using
//  any libraries, and with a timeout. This confirms whether the crystal
//  is actually oscillating.
//  Since this test does not depend on fallback mechanisms or other
//  indirect checks, its result is the most reliable.
// ============================================================================

#include <Arduino.h>

void setup() {
    Serial1.begin(115200);
    delay(300);
    Serial1.println("\n[LSE TEST] starting...");

    // Enable writes to the backup domain
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;

    // Reset once before starting oscillation, just to be safe
    RCC->BDCR &= ~RCC_BDCR_LSEON;
    delay(10);
    RCC->BDCR |= RCC_BDCR_LSEON;

    Serial1.println("[LSE TEST] LSEON set. Waiting for LSERDY (max ~5s)...");

    uint32_t t0 = millis();
    bool ready = false;
    while (millis() - t0 < 5000) {
        if (RCC->BDCR & RCC_BDCR_LSERDY) {
            ready = true;
            break;
        }
    }

    uint32_t elapsed = millis() - t0;

    if (ready) {
        Serial1.print("[LSE TEST] READY! LSE is oscillating. (");
        Serial1.print(elapsed);
        Serial1.println(" ms to stabilize)");
    } else {
        Serial1.println("[LSE TEST] TIMEOUT. LSE crystal is NOT oscillating.");
        Serial1.println("[LSE TEST] -> hardware issue (crystal, load caps, solder, or board defect)");
    }
}

void loop() {
    // Continuously display the current LSERDY state every 5 seconds
    // (to detect cases where oscillation starts later)
    Serial1.print("[LSE TEST] LSERDY=");
    Serial1.println((RCC->BDCR & RCC_BDCR_LSERDY) ? "1 (ready)" : "0 (not ready)");
    delay(5000);
}