# PIMA-node
Reads PIMA protocol from compatible energy meters (serial) and restransmits data in a Computourist style

# PIMA Protocol Library

A library to read data from energy meters compliant with Brazilian standards.

## Intruction

PIMA stands for _Protocolo para Infraestrutura de Medição Avançada_ (Protocol for Advanced Measurement Infrastructure, free translation)
and is defined as Brazilian standard for smart eneregy metering products comercialized in Brazil.

This library is designed for interpret PIMA packets read from unidirectional assynchronous serial available on digital energy meters
available in Brazil.

## Acknowloges

This started as a weekend project to monitor energy consumption and generation at home. I had access to a energy meter manufactured by
brazilian company [Nansen](http://www.nansen.com.br), model Vector PAR. This model is equiped with a assynchronous unidirectional serial
port that constantly outputs PIMA packets. Hence, this works with Vector meter but shall work with any other meter compliant with PIMA
standards.

Library only interprets packages and keep a register of last read values. Interfacing and reading of serial data is left out to the application
code in the same fashion as [Mikal Hart](https://github.com/mikalhart) [TinyGPS Library](https://github.com/mikalhart/TinyGPS).

Until now, only implemented energy quantities available on the Vector PAR output, other quantities shall be easy to add but I have no means
to test. I will gladly merge PRs that include new quantities. Again, Vector is unidirectional, so library only interpret received packets.

## PIMA Packets

A PIMA packet is composed as below:

| 2 Bytes | 5 Bytes | 1 Byte | 2 Bytes | N Bytes | 2 Bytes |
| --- | --- | --- | --- | --- | --- |
| Preamble | Identifier | Size | Escope + Index | Data | CRC16 |

- Preamble marks the start of packet and is composed by 2 hex bytes 0xAA 0x55
- Identifier is the serial number and is formatted as 5 bytes BCD (representing a 10 digit number)
- Size is the sum of bytes in Escope, Index and DATA (2 + N bytes)
- Data is the value for quantitie represented by Index
- CRC16 is the ANSI-CRC16 calculated over all bytes except Preamble and CRC itself

Only on Escope and Index are allowed per packet.

#### Escopes and Indexes

Until now, the following quantities are known to the library:

- Active Energy direct (KWh) - Escope 0x0A Index 0x02
- Inductive Reactive Energy direct (KVArh) - Escope 0x0A Index 0x07
- Capacitive Reactive Energy direct (KVArh) - Escope 0x0A Index 0x0C
- Active Energy reverse (KWh) - Escope 0x0A Index 0x51

While Vector PAR provide visual indication for capactive and inductive energy in reverse direction it does not transmit it over PIMA serial interface.
It shall be simple to add support to these quantities in the future versions of this library.

## References

- Nansen Vector PAR energy [meter manual](http://www.nansen.com.br/downloads/985cfb9777f1b9b67f53d5c9ac59f128.pdf) as avalaible in 2017-08-23.
- [2014] MIYAOKA, Edgar and MIYAWAKI, Mauro Kenji - [_"MEDIDOR DE ENERGIA ELETRÔNICO BIDIRECIONAL-SMART GRID"_](http://www.pucpr.br/arquivosUpload/5370721951436993327.pdf)
- [2009] DIÓRIO, Fernando Alvim - [_"PADRÕES PARA PROTOCOLO DE COMUNICAÇÃO"_](https://www.metering.com/wp-content/uploads/Fernando%20Alvim%20Diorio.pdf)

## Disclamair

This is not meant to be a reference for the PIMA protocol and it is not guarantee to work on all equipment (only tested on Vector PAR). If you are pursuing a reference,
please refers to the ABNT/INMETRO/ANEEL applicable standards and legislation.

**Energy meters are generally connect to mains BEFORE any breaker protection** DO NOT try to connect anything to it if you are not qualified for this kind of work.
I repeat: **DO NOT TRY TO CONNECT/INTERFACE WITH A ENERGY METER TIED TO MAINS**.

Also, be aware that in many countries (Brazil included), energy meter is placed in a cabinet sealed by energy company and break the seal is considered crime. YOU MAY
BE charged for stealing energy. My Vector PAR was not provided by energy company but installed inside my home just after the company meter so, I did not messed with company´s
sealed cabinet.

AUTHOR NOR ANY OTHER CONTRIBUTOR ARE RESPONSIBLE for any consequences of its use, including accidents of any kind caused, but not limited, by misconnection,
lack of knowledge in electricity or electronics, etc. **BE ADVISED**: energy kills! Steal energy is crime! Mess with company´s meter without proper authorization can be considered
crime and fines are applicable.

