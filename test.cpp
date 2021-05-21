#include "pch.h"
#include "voltajeRead.c"
#include "thermistor.c"
#include "adc2current.c"


/* 
* Prueba ADC a voltaje y el estado del voltaje de la fuente de
* Alimentacion DC
*/
TEST(TestGooglePruebaVoltaje, PruebaVoltajeH) {
	EXPECT_EQ(voltageErrorDet(2730), 1);
}
TEST(TestGooglePreubaVoltaje, PruebaVoltajeL) {
	EXPECT_EQ(voltageErrorDet(2357), 2);
}
TEST(TestGooglePreubaVoltaje, PruebaVoltajeN) {
	EXPECT_EQ(voltageErrorDet(2512), 0);
}

/*
* Temperatura
*/
TEST(TestGooglePreubaTemperatura, PruebaEstadoTempN) {
	EXPECT_EQ(testTempStatus(2047), 0);
}

TEST(TestGooglePreubaTemperatura, PruebaEstadoTempInvalida) {
	EXPECT_EQ(testTempStatus(50), 3);
}
TEST(TestGooglePreubaTemperatura, PruebaEstadoTempSobre) {
	EXPECT_EQ(testTempStatus(4000), 1);
}
TEST(TestGooglePreubaTemperatura, PruebaEstadoTempBajo) {
	EXPECT_EQ(testTempStatus(500), 2);
}

TEST(TestGooglePreubaTemperatura, PruebaTc) {
	EXPECT_NEAR(testTempValue(2047), 25.0, 0.03);
}
/*
* Test Corriente
*/
TEST(TestGooglePreubaCorriente, PruebaCorrienteN) {
	EXPECT_EQ(testCurrent(3840), 0);
}
TEST(TestGooglePreubaCorriente, PruebaCorrienteL) {
	EXPECT_EQ(testCurrent(3124), 2);
}
TEST(TestGooglePreubaCorriente, PruebaCorrienteH) {
	EXPECT_EQ(testCurrent(3996), 1);
}

