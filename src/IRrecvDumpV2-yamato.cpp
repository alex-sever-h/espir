/*
 * IRremoteESP8266: IRrecvDumpV2 - dump details of IR codes with IRrecv
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * Example circuit diagram:
 *  https://github.com/markszabo/IRremoteESP8266/wiki#ir-receiving
 * Changes:
 *   Version 0.2 April, 2017
 *     - Decode from a copy of the data so we can start capturing faster thus
 *       reduce the likelihood of miscaptures.
 * Based on Ken Shirriff's IrsendDemo Version 0.1 July, 2009, Copyright 2009 Ken Shirriff, http://arcfn.com
 */

#include <Arduino.h>

#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>

// An IR detector/demodulator is connected to GPIO pin 14(D5 on a NodeMCU
// board).
uint16_t RECV_PIN = 2;
uint16_t SEND_PIN = 5;
// As this program is a special purpose capture/decoder, let us use a larger
// than normal buffer so we can handle Air Conditioner remote codes.
uint16_t CAPTURE_BUFFER_SIZE = 1024;

IRrecv irrecv(RECV_PIN, CAPTURE_BUFFER_SIZE);
IRsend irsend(SEND_PIN);

decode_results results;  // Somewhere to store the results
irparams_t save;         // A place to copy the interrupt state while decoding.

void setup() {
  // Status message will be sent to the PC at 115200 baud
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  delay(500);  // Wait a bit for the serial connection to be establised.
  // Give the 'save' copy the same sized buffer.
  save.rawbuf = new uint16_t[irrecv.getBufSize()];
  if (save.rawbuf == NULL) {  // Check we allocated the memory successfully.
    Serial.printf("Could not allocate a %d buffer size for the save buffer.\n"
                  "Try a smaller size for CAPTURE_BUFFER_SIZE.\nRebooting!",
                  irrecv.getBufSize());
    ESP.restart();
  }

  irrecv.enableIRIn();  // Start the receiver
  irsend.begin();
}

// Display encoding type
//
void encoding(decode_results *results) {
  switch (results->decode_type) {
    default:
    case UNKNOWN:      Serial.print("UNKNOWN");       break;
    case NEC:          Serial.print("NEC");           break;
    case NEC_LIKE:     Serial.print("NEC (non-strict)");  break;
    case SONY:         Serial.print("SONY");          break;
    case RC5:          Serial.print("RC5");           break;
    case RC5X:         Serial.print("RC5X");          break;
    case RC6:          Serial.print("RC6");           break;
    case RCMM:         Serial.print("RCMM");          break;
    case DISH:         Serial.print("DISH");          break;
    case SHARP:        Serial.print("SHARP");         break;
    case JVC:          Serial.print("JVC");           break;
    case SANYO:        Serial.print("SANYO");         break;
    case SANYO_LC7461: Serial.print("SANYO_LC7461");  break;
    case MITSUBISHI:   Serial.print("MITSUBISHI");    break;
    case SAMSUNG:      Serial.print("SAMSUNG");       break;
    case LG:           Serial.print("LG");            break;
    case WHYNTER:      Serial.print("WHYNTER");       break;
    case AIWA_RC_T501: Serial.print("AIWA_RC_T501");  break;
    case PANASONIC:    Serial.print("PANASONIC");     break;
    case DENON:        Serial.print("DENON");         break;
    case COOLIX:       Serial.print("COOLIX");        break;
    case YAMATO:       Serial.print("YAMATO");        break;

  }
  if (results->repeat) Serial.print(" (Repeat)");
}

// Dump out the decode_results structure.
//
void dumpInfo(decode_results *results) {
  if (results->overflow)
    Serial.printf("WARNING: IR code too big for buffer (>= %d). "
                  "These results shouldn't be trusted until this is resolved. "
                  "Edit & increase CAPTURE_BUFFER_SIZE.\n",
                  CAPTURE_BUFFER_SIZE);

  // Show Encoding standard
  Serial.print("Encoding  : ");
  encoding(results);
  Serial.println("");

  // Show Code & length
  Serial.print("Code      : ");
  serialPrintUint64(results->value, 16);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.println(" bits)");
}

uint16_t getCookedLength(decode_results *results) {
  uint16_t length = results->rawlen - 1;
  for (uint16_t i = 0; i < results->rawlen - 1; i++) {
    uint32_t usecs = results->rawbuf[i] * RAWTICK;
    // Add two extra entries for multiple larger than UINT16_MAX it is.
    length += (usecs / UINT16_MAX) * 2;
  }
  return length;
}

// Dump out the decode_results structure.
//
void dumpRaw(decode_results *results) {
  // Print Raw data
  Serial.print("Timing[");
  Serial.print(results->rawlen - 1, DEC);
  Serial.println("]: ");

  for (uint16_t i = 1; i < results->rawlen; i++) {
    if (i % 100 == 0)
      yield();  // Preemptive yield every 100th entry to feed the WDT.
    if (i % 2 == 0) {  // even
      Serial.print("-");
    } else {  // odd
      Serial.print("   +");
    }
    Serial.printf("%6d", results->rawbuf[i] * RAWTICK);
    if (i < results->rawlen - 1)
      Serial.print(", ");  // ',' not needed for last one
    if (!(i % 8)) Serial.println("");
  }
  Serial.println("");  // Newline
}

// Dump out the decode_results structure.
//
void dumpCode(decode_results *results) {
  // Start declaration
  Serial.print("uint16_t ");               // variable type
  Serial.print("rawData[");                // array name
  Serial.print(getCookedLength(results), DEC);  // array size
  Serial.print("] = {");                   // Start declaration

  // Dump data
  for (uint16_t i = 1; i < results->rawlen; i++) {
    uint32_t usecs;
    for (usecs = results->rawbuf[i] * RAWTICK;
         usecs > UINT16_MAX;
         usecs -= UINT16_MAX)
      Serial.printf("%d, 0", UINT16_MAX);
    Serial.print(usecs, DEC);
    if (i < results->rawlen - 1)
      Serial.print(", ");  // ',' not needed on last one
    if (i % 2 == 0) Serial.print(" ");  // Extra if it was even.
  }

  // End declaration
  Serial.print("};");  //

  // Comment
  Serial.print("  // ");
  encoding(results);
  Serial.print(" ");
  serialPrintUint64(results->value, HEX);

  // Newline
  Serial.println("");

  // Now dump "known" codes
  if (results->decode_type != UNKNOWN) {
    // Some protocols have an address &/or command.
    // NOTE: It will ignore the atypical case when a message has been decoded
    // but the address & the command are both 0.
    if (results->address > 0 || results->command > 0) {
      Serial.print("uint32_t address = 0x");
      Serial.print(results->address, HEX);
      Serial.println(";");
      Serial.print("uint32_t command = 0x");
      Serial.print(results->command, HEX);
      Serial.println(";");
    }

    // All protocols have data
    Serial.print("uint64_t data = 0x");
    serialPrintUint64(results->value, 16);
    Serial.println(";");
  }
}


uint8_t something[] = {0x23, 0xCB, 0x26, 0x01, 0x00, // never changes
                       0x24, 0x03, 0x06, 0x22, 0x00, 0x00, 0x03, 0x04,
                       0x6B }; // CRC

uint16_t rawData[227] = {3846, 1392,  618, 1138,  616, 1138,  616, 442,  590, 442,  590, 422,  612, 1136,  614, 444,  590, 444,  590, 1140,  614, 1138,  616, 442,  588, 1138,  616, 444,  588, 442,  590, 1138,  614, 1138,  620, 442,  590, 1140,  614, 1136,  616, 444,  590, 442,  590, 1136,  616, 440,  592, 420,  614, 1136,  616, 442,  590, 442,  590, 442,  588, 442,  590, 444,  590, 442,  588, 424,  614, 440,  590, 442,  590, 442,  590, 442,  588, 442,  590, 442,  590, 442,  590, 444,  592, 442,  590, 444,  590, 1138,  614, 442,  590, 442,  590, 1136,  616, 440,  590, 424,  612, 1138,  614, 1138,  616, 444,  590, 442,  590, 440,  590, 442,  590, 442,  590, 444,  592, 442,  592, 1136,  618, 1136,  616, 442,  590, 442,  590, 442,  592, 440,  592, 422,  612, 442,  592, 1138,  616, 420,  612, 440,  592, 442,  590, 1138,  616, 442,  590, 444,  592, 442,  588, 444,  590, 442,  588, 442,  590, 442,  590, 440,  592, 418,  614, 424,  612, 442,  592, 440,  590, 440,  592, 440,  592, 440,  592, 442,  590, 442,  590, 424,  612, 1136,  618, 1138,  616, 440,  592, 442,  590, 440,  592, 442,  590, 442,  590, 422,  614, 440,  592, 442,  590, 1138,  616, 440,  592, 440,  590, 440,  592, 442,  590, 444,  592, 1136,  618, 1136,  616, 440,  628, 1102,  650, 406,  592, 1136,  616, 1136,  654, 406,  618};  // YAMATO 0


// The repeating section of the code
//
void loop() {
  // Check if the IR code has been received.
#if 0
  if (irrecv.decode(&results, &save)) {
    dumpInfo(&results);           // Output the results
#if 1
    dumpRaw(&results);            // Output the results in RAW format
    dumpCode(&results);           // Output the results as source code
#endif
    Serial.println("");           // Blank line between entries
  }
#endif

  delay(2000);  // Wait a bit for the serial connection to be establised.
  Serial.println("send YAMATO");
  //irsend.sendRaw(rawData, 227, 38);  // Send a raw data capture at 38kHz.

  irsend.sendYamato(something);
}
