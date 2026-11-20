#include <Arduino.h>
#include "protocol.h"

uint8_t led_state = LOW;
uint32_t serial1_bauds = SERIAL_BAUDS;
uint32_t serial_monitor_bauds = SERIAL_MONITOR_BAUDS;
uint8_t currentI2C_address = 0;

#define COMMAND_BUFFER_LENGTH 64
#define MAX_ARGS 10

char commandBuffer[COMMAND_BUFFER_LENGTH + 1];
int bufferIndex = 0;

char *currentCommandArgs[MAX_ARGS + 1];

struct Command {
  const char* name;
  void (*handler)(int argc, char *argv[]);
  const char* help;
};

/* ---------------------------- utilidades ---------------------------- */

const Meassure* findMeassure(byte protocolMeassure) {
  for(const Meassure &meassure: meassures) {
    if(meassure.repr == protocolMeassure) {
      return &meassure;
    }
  }
  return nullptr;
}

void toggleLed() {
  digitalWrite(LED_BUILTIN, led_state); led_state = (led_state + 1) & 0x1;
}

void printAddress(byte address) {
  SerialUSB.print("0x");
  if(address < 0x10) SerialUSB.print('0');
  SerialUSB.print(address, HEX);
}

void printMeassure(const byte *data) {
  // data = [med_hi, med_lo, unit]
  uint16_t value = ((uint16_t) data[0] << 8) | data[1];
  const Meassure *m = findMeassure(data[2]);
  SerialUSB.print(value);
  SerialUSB.print(' ');
  SerialUSB.println(m ? m->symbol : "(unidad desconocida)");
}

const char* errorDescription(byte code) {
  switch(code) {
    case ERR_UNKNOWN_SENSOR: return "sensor no registrado en el dispositivo";
    case ERR_I2C:            return "fallo de comunicacion I2C con el sensor";
    case ERR_BAD_ARGS:       return "argumentos invalidos";
    case ERR_UNKNOWN_OP:     return "operacion no soportada";
    case ERR_TOO_SOON:       return "aun no ha pasado el retardo minimo entre disparos";
    default:                 return "error desconocido";
  }
}

// Medida periodica que llega sin haberla pedido
void printPeriodic(const Frame &frame) {
  if(frame.len < 4) return;
  SerialUSB.print("[periodico] sensor ");
  printAddress(frame.body[0]);
  SerialUSB.print(": ");
  printMeassure(frame.body + 1);
}

// Envia una peticion y espera su respuesta. Devuelve true solo si es un ACK.
bool transact(byte opcode, byte len, const byte *body, Frame &out) {
  sendFrame(opcode, len, body);

  // Si mientras esperamos llega una medida periodica, se muestra y se sigue esperando
  do {
    if(!readFrame(out)) {
      SerialUSB.println("<-- TIMEOUT!! El dispositivo sensor no responde");
      return false;
    }
    if(out.opcode == OP_MEASSURE) printPeriodic(out);
  } while(out.opcode == OP_MEASSURE);

  toggleLed();

  if(out.opcode == OP_NACK) {
    SerialUSB.print("<-- ERROR: ");
    SerialUSB.println(out.len > 0 ? errorDescription(out.body[0]) : "sin detalle");
    return false;
  }
  if(out.opcode != OP_ACK) {
    SerialUSB.print("<-- Respuesta inesperada, opcode ");
    printAddress(out.opcode);
    SerialUSB.println();
    return false;
  }
  return true;
}

// Convierte un argumento decimal a uint16 en [1, 65535]
bool parseMilliseconds(const char *arg, uint16_t &result) {
  char *end;
  unsigned long v = strtoul(arg, &end, DEC);

  if(end == arg || *end != '\0') {
    SerialUSB.print("El valor '");
    SerialUSB.print(arg);
    SerialUSB.println("' no es un numero decimal valido");
    return false;
  }
  if(v == 0 || v > 0xFFFF) {
    SerialUSB.print("El valor '");
    SerialUSB.print(arg);
    SerialUSB.println("' no esta en el rango valido [1-65535] ms");
    return false;
  }
  result = (uint16_t) v;
  return true;
}

/* ----------------------------- comandos ----------------------------- */

void cmdHelp(int argc, char *argv[]);
void cmdUs(int argc, char *argv[]);
void cmdOneShot(int argc, char *argv[]);
void cmdOn(int argc, char *argv[]);
void cmdOff(int argc, char *argv[]);
void cmdUnit(int argc, char *argv[]);
void cmdDelay(int argc, char *argv[]);
void cmdStatus(int argc, char *argv[]);

const Command commands[] = {
  {"help", cmdHelp, "help\n\tMuestra esta ayuda"},
  {"us", cmdUs,
    "us\n\tLista los sensores SRF02 disponibles en el dispositivo sensor\n"
    "us <srf02> {one-shot | on <period_ms> | off | unit {inc|cm|ms} | delay <ms> | status | help}\n"
    "\t<srf02> es la direccion I2C del sensor en hexadecimal (ej. E0 o 0xE0)"}
};

const Command subcommands[] = {
  {"one-shot", cmdOneShot, "Realiza un unico disparo y muestra la medida"},
  {"on", cmdOn, "on <period_ms>: dispara periodicamente cada <period_ms> ms"},
  {"off", cmdOff, "Detiene el disparo periodico"},
  {"unit", cmdUnit, "unit {inc | cm | ms}: cambia la unidad de medida"},
  {"delay", cmdDelay, "delay <ms>: retardo minimo entre dos disparos consecutivos"},
  {"status", cmdStatus, "Muestra direccion, retardo, unidad y estado periodico"}
};

void printCommandHelp(const Command &command) {
  SerialUSB.print("  ");
  SerialUSB.println(command.name);
  SerialUSB.print("\t");
  SerialUSB.println(command.help);
}

void cmdHelp(int argc, char *argv[]){
  if(argc > 1) {
    SerialUSB.println("El uso de 'help' no requiere de argumentos. Ingresa 'help'");
    return;
  }

  SerialUSB.println("Comandos permitidos:");
  for(const Command &command: commands) {
    SerialUSB.println(command.help);
  }
  SerialUSB.println("Subcomandos de 'us <srf02>':");
  for(const Command &command: subcommands) {
    printCommandHelp(command);
  }
  SerialUSB.println("Las medidas de los sensores en modo periodico se muestran de forma asincrona.");
}

void listSensors() {
  Frame out;
  if(!transact(OP_US, 0, nullptr, out)) return;

  if(out.len < 1 || out.body[0] > out.len - 1) {
    SerialUSB.println("<-- Respuesta mal formada");
    return;
  }
  byte n = out.body[0];
  if(n == 0) {
    SerialUSB.println("No hay sensores SRF02 disponibles");
    return;
  }
  SerialUSB.println("Sensores SRF02 disponibles:");
  for(byte i = 0; i < n; i++) {
    SerialUSB.print("  ");
    SerialUSB.print(i + 1);
    SerialUSB.print(". ");
    printAddress(out.body[1 + i]);
    SerialUSB.println();
  }
}

void cmdUs(int argc, char *argv[]) {
  if(argc == 1) {
    listSensors();
    return;
  }

  char *end;
  unsigned long v = strtoul(argv[1], &end, HEX);

  if(end == argv[1] || *end != '\0') {
    SerialUSB.print("El valor '");
    SerialUSB.print(argv[1]);
    SerialUSB.println("' no es un valor hexadecimal valido");
    return;
  }
  if(v > 0xFF) {
    SerialUSB.print("El valor '");
    SerialUSB.print(argv[1]);
    SerialUSB.println("' no esta en el rango valido [0x0-0xFF]");
    return;
  }

  if(argc < 3 || strcmp("help", argv[2]) == 0) {
    SerialUSB.println("Elija un subcomando a ejecutar entre los siguientes: ");
    for(const Command &command: subcommands) {
      printCommandHelp(command);
    }
    return;
  }

  currentI2C_address = (uint8_t) v;
  for(const Command &command: subcommands) {
    if(strcmp(argv[2], command.name) == 0) {
      command.handler(argc - 3, argv + 3);
      return;
    }
  }
  SerialUSB.print("No se reconoce el siguiente subcomando: '");
  SerialUSB.print(argv[2]);
  SerialUSB.println("'");
  SerialUSB.println("Para mas informacion ejecute us <srf02> help");
}

void cmdOneShot(int argc, char *argv[]) {
  if(argc > 0) {
    SerialUSB.println("El comando one-shot no precisa de argumentos");
    return;
  }

  byte body[] = { currentI2C_address };
  Frame out;
  if(!transact(OP_ONE_SHOT, sizeof(body), body, out)) return;

  if(out.len < 4) {
    SerialUSB.println("<-- Respuesta mal formada");
    return;
  }
  SerialUSB.print("<-- Medida del sensor ");
  printAddress(out.body[0]);
  SerialUSB.print(": ");
  printMeassure(out.body + 1);
}

void cmdOn(int argc, char *argv[]) {
  if(argc < 1) {
    SerialUSB.println("El comando on necesita el numero de milisegundos entre muestras");
    return;
  }else if (argc > 1) {
    SerialUSB.println("Sobran argumentos. Ingrese unicamente los milisegundos");
    return;
  }

  uint16_t milliseconds;
  if(!parseMilliseconds(argv[0], milliseconds)) return;

  byte body[] = { currentI2C_address, (byte) (milliseconds >> 8), (byte) (milliseconds & 0xFF) };
  Frame out;
  if(transact(OP_ON, sizeof(body), body, out)) {
    SerialUSB.print("<-- Muestreo periodico activado cada ");
    SerialUSB.print(milliseconds);
    SerialUSB.println(" ms");
  }
}

void cmdOff(int argc, char *argv[]) {
  if(argc > 0) {
    SerialUSB.println("El comando off no precisa de argumentos");
    return;
  }

  byte body[] = { currentI2C_address };
  Frame out;
  if(transact(OP_OFF, sizeof(body), body, out)) {
    SerialUSB.println("<-- Se ha cancelado el muestreo periodico");
  }
}

void cmdUnit(int argc, char *argv[]) {
  if(argc < 1) {
    SerialUSB.println("El comando unit necesita una unidad de medida de entre las siguientes:");
    for(const Meassure &meassure: meassures) {
      SerialUSB.print("  ");
      SerialUSB.println(meassure.name);
    }
    return;
  }else if(argc > 1) {
    SerialUSB.println("Sobran argumentos. Ingrese unicamente la medida que desea");
    return;
  }

  for(const Meassure &meassure: meassures) {
    if(strcmp(meassure.name, argv[0]) == 0) {
      byte body[] = { currentI2C_address, meassure.repr };
      Frame out;
      if(transact(OP_UNIT, sizeof(body), body, out)) {
        SerialUSB.print("<-- La unidad se ha cambiado a ");
        SerialUSB.println(meassure.name);
      }
      return;
    }
  }

  SerialUSB.print("Unidad '");
  SerialUSB.print(argv[0]);
  SerialUSB.println("' no valida. Use inc, cm o ms");
}

void cmdDelay(int argc, char *argv[]){
  if(argc < 1) {
    SerialUSB.println("El comando delay necesita el numero minimo de milisegundos entre muestras");
    return;
  }else if (argc > 1) {
    SerialUSB.println("Sobran argumentos. Ingrese unicamente los milisegundos");
    return;
  }

  uint16_t milliseconds;
  if(!parseMilliseconds(argv[0], milliseconds)) return;

  byte body[] = { currentI2C_address, (byte) (milliseconds >> 8), (byte) (milliseconds & 0xFF) };
  Frame out;
  if(transact(OP_DELAY, sizeof(body), body, out)) {
    SerialUSB.print("<-- Retardo minimo entre disparos establecido a ");
    SerialUSB.print(milliseconds);
    SerialUSB.println(" ms");
  }
}

void cmdStatus(int argc, char *argv[]){
  if(argc > 0) {
    SerialUSB.println("El comando status no precisa de argumentos");
    return;
  }

  byte body[] = { currentI2C_address };
  Frame out;
  if(!transact(OP_STATUS, sizeof(body), body, out)) return;

  if(out.len < 7) {
    SerialUSB.println("<-- Respuesta mal formada");
    return;
  }

  uint16_t minDelay = ((uint16_t) out.body[1] << 8) | out.body[2];
  const Meassure *unit = findMeassure(out.body[3]);
  bool periodic = out.body[4] != 0;
  uint16_t period = ((uint16_t) out.body[5] << 8) | out.body[6];

  SerialUSB.println("<-- Estado del sensor:");
  SerialUSB.print("  Direccion I2C:     ");
  printAddress(out.body[0]);
  SerialUSB.println();
  SerialUSB.print("  Retardo minimo:    ");
  SerialUSB.print(minDelay);
  SerialUSB.println(" ms");
  SerialUSB.print("  Unidad de medida:  ");
  SerialUSB.println(unit ? unit->name : "desconocida");
  SerialUSB.print("  Disparo periodico: ");
  if(periodic) {
    SerialUSB.print("activado (cada ");
    SerialUSB.print(period);
    SerialUSB.println(" ms)");
  }else {
    SerialUSB.println("desactivado");
  }
}

/* ------------------------- interprete y loop ------------------------- */

int parseCommandLine(char *line, char *argv[]) {
  int argc = 0;

  char *token = strtok(line, " \t");

  while(token != nullptr && argc < MAX_ARGS) {
    argv[argc++] = token;
    token = strtok(nullptr, " \t");
  }

  argv[argc] = nullptr;

  return argc;
}

void handleCommand() {
  int argc = parseCommandLine(commandBuffer, currentCommandArgs);

  if(argc == 0) {
    return;
  }

  for(const Command &command: commands) {
    if(strcmp(command.name, currentCommandArgs[0]) == 0) {
      command.handler(argc, currentCommandArgs);
      return;
    }
  }

  SerialUSB.print("No se reconoce el comando: ");
  SerialUSB.println(currentCommandArgs[0]);
  SerialUSB.println("Para mas informacion ingresa 'help'.");
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  toggleLed();

  SerialUSB.begin(serial_monitor_bauds);
  while(!SerialUSB){}
  Serial1.begin(serial1_bauds);

  SerialUSB.println("******************** supervisor SRF02 ********************");
  SerialUSB.println("Escriba 'help' para ver los comandos disponibles");
}

void loop() {
  // Entrada del usuario por el monitor serie
  if(SerialUSB.available() > 0) {
    char data = SerialUSB.read();
    if(data == '\n' || data == '\r') {
      if(bufferIndex > 0) {
        commandBuffer[bufferIndex] = '\0';
        bufferIndex = 0;
        SerialUSB.println();
        handleCommand();
      }
    }else if(data == '\b' || data == 0x7F) {
      if (bufferIndex > 0) {
        bufferIndex--;
        SerialUSB.print("\b \b");
      }
    }else if(bufferIndex < COMMAND_BUFFER_LENGTH) {
      commandBuffer[bufferIndex++] = data;
      SerialUSB.print(data);
    }
  }

  // Tramas asincronas del dispositivo sensor (medidas periodicas)
  if(Serial1.available() > 0) {
    Frame frame;
    if(readFrame(frame) && frame.opcode == OP_MEASSURE) {
      toggleLed();
      printPeriodic(frame);
    }
  }
}
