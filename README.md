# riscv_wasemu (mini-rv32ima)

This is an extention / fork of cnlohr's [mini-rv32ima](https://github.com/cnlohr/mini-rv32ima) project.

<p align = "center">
    <img src="https://raw.githubusercontent.com/gili-yankovitch/mini-rv32ima/refs/heads/master/images/sos-wasm.png" height="500">
</p>

## What

The current objective of this specific repo is to be a WASM clone for the ch32v003 chip. It currently implements *some* of the functionality of the chip, but overall works great..
 * Implements a RISC-V **rv32ima** - From original project
 * Extends RV implementation to support RVC
 * Implements: SYSTICK, RCC, USART, SPI, W25Q Flash device
 * Has **no dependencies**, not even libc.
 * Is **easily extensible**.  So you can easily add CSRs, instructions, MMIO, etc!
 * Is human-readable and in **basic C** code.
 * Is "**incomplete**" in that it doesn't have stuff that wasn't originally required.
 * Supports both cmdline as well as WASM mode

## Why

This is part of the [SwordOfSecrets](https://github.com/gili-yankovitch/SwordOfSecrets) project, providing an emulated environemnt to solve the challenge.

## How

WSL / Linux:
 * Clone this rpo.
 * Install [emscripten](https://emscripten.org/docs/getting_started/downloads.html)
 * `make`

## Running standalone

Contrast to the WASM version, the CLI version accepts a `.bin` image. I have mainly tested it with cnlohr's `ch32v003fun` framwork, but this project was built closely with the [respective](http://mcu.cz/images_articles/5007-CH32V003Reference-Manual.PDF) [datasheets](https://riscv.org/wp-content/uploads/2017/05/riscv-spec-v2.2.pdf), so it should be generic.
