# Progetto di Sistemi Operativi

Assegnazione:
- All projects involvig Arduino should include some sort of serial
  communication with the host.
  The protocol should be binary, and data integrity should be ensured by
  a checksum.

  Meaning: a project of arduino includes a "server" (arduino part),
  and a client (PC part). These are two different programs.
  No cutecom needed for using arduino.

- Use of resources:
  Use at least one interrupt a timer and an I/O port

SmartHouse [2-3 w/BT]
   Arduino: Build a smart switch/dimmer/controller that can be controlled
            by bluetooth. We don't use bluetooth, we use a serial first.

            The arduino should support:
            8 switches/dimmers (simulate them with LEDS). (interruttori 4x e led 4x )
            8 ADC channels
            8 Digital inputs   (Uso i sensori di temperatura 1x digital_pin/ea, led rgb 3x digital_pin_pwm/ea)

            Define a protocol to
            - Configure the device (give it a "name"),
              set which channels are used, and what is the name of each channel

            - Send commands to the device (based on the "name");
              Upon correct execution of a command, the device should send back
              an "ack" message.

            - receive readings from the device (based on the "name");


   From PC: Implement a "controller" program that can baptize the device,
            and interact with it, from command line
            Il controllore è la scheda Arduino. I canali sono le porte a cui connetto i dispositivi.
            I valori sono ad esempio temp_min e temp_max per il sensore di temperatura.
            Query_channels mi dice quali canali sono occupati e da chi.

            eg

            $> smart_house_host set_name kitchen_controller
            $> smart_house set_channel_name kitchen_controller switch_0 "top_fridge"
            $> smart_house set_channel_value "top_fridge" 1
            $> smart_house kitchen_controller query_channels


Implementazione:

[Arduino app]
Dispositivo in attesa del collegamento del PC.
PC invia un comando e la scheda risponde in modo appropriato (ack di ricezione ed esecuzione comando)
Design choice: un solo Sender (PC) alla volta può connettersi sulla Porta Seriale 0 della scheda per comunicare.
Non sarà necessario verificare da quale fonte provengono i pacchetti.

Protocollo: invio di pacchetti a dimensione fissa da 32 Byte.
1B Command(6bit cmd + 2bit packet number) + 1B checksum + 1B size(5bit size + 3bit sizecksum) + 29B payload (padding)
Checksum sull'intero pacchetto o solo sul comando (se il comando non necessita della payload)

Comandi: ogni comanda è codificato usando 6 bit (MSB) cui vanno aggiunti 2 bit di pacchetto in coda (lsb)

La CPU della scheda è in sleep_mode fintanto che non arriva un intero pacchetto, in tal caso viene controllato per verificarne la correttezza (sequence number, size control bits e checksum) ed eventualmente processato.
La scheda risponde con un ack se il check è andato a buon fine ed il pacchetto è stato processato, con un nack se non ha superato il check, con ucmd (unknown command) se il comando non è stato riconosciuto.

Comandare la board:
1) install()
2*) setChannelName() e/o queryChannels()
3*) funzioni custom dei device per impostarli (Es: set minTemp() per sensore di temperatura) 
