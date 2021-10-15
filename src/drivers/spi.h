#include "hardware.h"

void spi_init(void);


void SPIM_CSLow(void);

void SPIM_CSHigh(void);

void SPIM_TXEn(SPI_TypeDef *SPIx);

void SPIM_TXDisable(SPI_TypeDef *SPIx);

void SPIM_RXEn(SPI_TypeDef *SPIx);

void SPIM_RXDisable(SPI_TypeDef *SPIx);

unsigned int SPI_RW(unsigned char tx_data);

void SPIM2_CSLow(void);

void SPIM2_CSHigh(void);

unsigned int SPI2_RW(unsigned char tx_data);

void spi2_init(void);


