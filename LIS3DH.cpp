/*
 * mbed library program
 *  LIS3DH MEMS motion sensor: 3-axis "nano" accelerometer, made by STMicroelectronics
 *      http://www.st-japan.co.jp/web/jp/catalog/sense_power/FM89/SC444/PF250725
 *
 * Copyright (c) 2014,'15 Kenji Arai / JH1PJL
 *  http://www.page.sannet.ne.jp/kenjia/index.html
 *  http://mbed.org/users/kenjiArai/
 *      Created: July      14th, 2014
 *      Revised: December  12th, 2015
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "LIS3DH.h"

LIS3DH::LIS3DH (std::shared_ptr<SPI> _spi, DigitalOut _cs, uint8_t data_rate, uint8_t fullscale) : spi(_spi), cs(_cs)
{
    spi->frequency(4000000);
    init (data_rate, fullscale);
}

LIS3DH::LIS3DH (std::shared_ptr<SPI> _spi, DigitalOut _cs) : spi(_spi), cs(_cs)
{
    spi->frequency(4000000);
    init (LIS3DH_DR_NR_LP_50HZ, LIS3DH_FS_8G);
}

LIS3DH::LIS3DH(std::shared_ptr<SPI> _spi, std::shared_ptr<BusOut> _csBus, uint8_t _csAddr, uint8_t data_rate, uint8_t fullscale)
    : spi(_spi)
    , csBus(_csBus)
    , csAddr(_csAddr)
    , cs(NC)
{
    spi->frequency(4000000);
    init (data_rate, fullscale);
}


void LIS3DH::readRegs(uint8_t addr, char * data, int len) {
    // cs = 1 ;
    // wait_us(1000);
    for (int i = 0 ; i < len ; i++ ) {    
        tx_buf[0] = (addr+i)|0x80;
        activateCS();
        spi->write(tx_buf, 1, rx_buf, 2) ;  // specify address to read
        deactivateCS();
        // printf("\r\nspi read %i got %i %i %i %i", addr+i, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
        data[i] = rx_buf[1];
    } 
    // spi->write(0x00) ; // to terminate read mode
    // wait_us(1000);
    // cs = 0 ;
}

void LIS3DH::writeRegs(uint8_t * data, int len) {
    activateCS();
    for (int i = 0 ; i < len ; i++ ) {
        spi->write(data[i]) ;
    }
    deactivateCS();
}

void LIS3DH::write_reg(uint8_t addr, uint8_t data8)
{
    uint8_t data[2] ;
    data[0] = addr ;
    data[1] = data8 ;
    writeRegs(data, 2) ;
}

uint8_t LIS3DH::read_reg(uint8_t addr)
{
    char data[1] ;    
    readRegs(addr, data, 1) ;
    return( data[0] ) ;
}

void LIS3DH::write16(uint8_t addr, uint16_t data16)
{
    uint8_t data[3] ;
    data[0] = addr ;
    data[1] = (data16 >> 8) & 0xFF ;
    data[2] = data16 & 0xFF ;
    writeRegs(data, 3) ;
}

uint16_t LIS3DH::read16(uint8_t addr)
{
    char data[2] ;
    uint16_t value = 0 ;
    readRegs(addr, data, 2) ;
    value = (data[0] << 8) | data[1] ;
    return( value ) ;
}


void LIS3DH::init(uint8_t data_rate, uint8_t fullscale)
{
    spi->frequency(4000000);
    dt[0] = read_reg(LIS3DH_WHO_AM_I);
    if (dt[0] == I_AM_LIS3DH) {
        acc_ready = 1;
    } else {
        acc_ready = 0;
        return;     // acc chip is NOT on I2C line then terminate
    }
    
    write_reg(LIS3DH_CTRL_REG1, (0x07|(data_rate << 4))); //  Reg.1
    write_reg(LIS3DH_CTRL_REG4, (0x08|(fullscale << 4))); //  Reg.4
    
    switch (fullscale) {
        case LIS3DH_FS_2G:
            fs_factor = LIS3DH_SENSITIVITY_2G;
            break;
        case LIS3DH_FS_4G:
            fs_factor = LIS3DH_SENSITIVITY_4G;
            break;
        case LIS3DH_FS_8G:
            fs_factor = LIS3DH_SENSITIVITY_8G;
            break;
        case LIS3DH_FS_16G:
            fs_factor = LIS3DH_SENSITIVITY_16G;
            break;
        default:
            ;
    }
}

void LIS3DH::read_mg_data(float *dt_usr)
{
    char data[6];

    if (acc_ready == 0) {
        dt_usr[0] = 0;
        dt_usr[1] = 0;
        dt_usr[2] = 0;
        return;
    }
    readRegs(LIS3DH_OUT_X_L, data, 6); 
    // change data type
#if OLD_REV // Fixed bugs -> (1) unit is not mg but g (2) shift right 4bit = /16
    dt_usr[0] = float(short((data[1] << 8) | data[0])) * fs_factor / 15;
    dt_usr[1] = float(short((data[3] << 8) | data[2])) * fs_factor / 15;
    dt_usr[2] = float(short((data[5] << 8) | data[4])) * fs_factor / 15;
#else
    dt_usr[0] = float(short((data[1] << 8) | data[0]) >> 4) * fs_factor;
    dt_usr[1] = float(short((data[3] << 8) | data[2]) >> 4) * fs_factor;
    dt_usr[2] = float(short((data[5] << 8) | data[4]) >> 4) * fs_factor;
#endif
}

void LIS3DH::read_data(float *dt_usr)
{
    char data[6];

    if (acc_ready == 0) {
        dt_usr[0] = 0;
        dt_usr[1] = 0;
        dt_usr[2] = 0;
        return;
    }
    readRegs(LIS3DH_OUT_X_L, data, 6); 
    // change data type
#if OLD_REV // Fixed bugs -> shift right 4bit = /16 (not /15)
    dt_usr[0] = float(short((data[1] << 8) | data[0])) * fs_factor / 15 * GRAVITY;
    dt_usr[1] = float(short((data[3] << 8) | data[2])) * fs_factor / 15 * GRAVITY;
    dt_usr[2] = float(short((data[5] << 8) | data[4])) * fs_factor / 15 * GRAVITY;
#else
    dt_usr[0] = float(short((data[1] << 8) | data[0]) >> 4) * fs_factor * GRAVITY;
    dt_usr[1] = float(short((data[3] << 8) | data[2]) >> 4) * fs_factor * GRAVITY;
    dt_usr[2] = float(short((data[5] << 8) | data[4]) >> 4) * fs_factor * GRAVITY;
#endif
}

uint8_t LIS3DH::read_id()
{
    dt[0] = read_reg(LIS3DH_WHO_AM_I);
    return dt[0];
}

uint8_t LIS3DH::data_ready()
{
    if (acc_ready == 1) { //device initialized correctly
        dt[0] = read_reg(LIS3DH_STATUS_REG_AUX);
        if (!(dt[0] & 0x01)) {
            return 0;
        }
    }
    return 1;
}

void LIS3DH::set_frequency(int hz)
{
    spi->frequency(hz);
}