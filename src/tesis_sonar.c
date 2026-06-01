/////////////////////////////////////////////////////
//LIBRERIAS

//Librerias STD
#include <stdio.h>
#include <unistd.h>
#include <math.h>

//Librerias propias de MicroRos
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>

//Librerias propias de raspi pico
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

//I2C Para IMU
#include "hardware/i2c.h"
#include "pico/binary_info.h"

//Librerias de mensajes
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>

/////////////////////////////////////////////////////
//CONSTANTES

//Tiempo de INIT
#define TON_INIT 55
#define TOFF_INIT 20

//Numero de sensores por defecto
#define MAX_SENSORES 12

//Distancia maxima y minima por defecto (Mts)
#define MAX_DIST 5 
#define MIN_DIST 0.4 

#define HISTERESIS 0.05 //En mts

#define UMBRAL_SEGURIDAD 0.5

//Lobulo del sensor
#define FIELD_VIEW 0.244346

//Frecuencias de Publicacion (Hz)
#define FREC_SONAR 1
#define FREC_IMU 10

#define FREC_CALLBACK 20

#define MAX_FREC_SONAR 2
#define MAX_FREC_IMU 20

#define MIN_FREC_SONAR 0.5
#define MIN_FREC_IMU 5

//GPIO
#define LED_PIN 21
#define LED_INTEGRADO 25
#define ECHO 20 
#define INIT 19 

#define SEG_1 16
#define SEG_2 17

//UART
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_USADA uart0
#define BAUD_RATE 921600

//ROS
#define NODE_NAME "mod_sens"
#define NAMESPACE_NAME "modsens"
#define TOPIC_NAME_SONAR "range"
#define TOPIC_NAME_IMU "imu"
#define TOPIC_NAME_CANT_SENSORES "cant_sensores"
#define TOPIC_NAME_DIST_SEG "dist_seguridad"

#define TIMEOUT_MS 1000
#define INTENTOS_CONN 10

//I2C
#define I2C_PORT i2c1
#define SDA_PIN 2
#define SCL_PIN 3
#define FREQ_I2C 100 * 1000

#define LSM6DS3_ADDR 0x6B
#define LSM6DS3_WHO_AM_I  0x0F
#define LSM6DS3_CTRL1_XL  0x10
#define LSM6DS3_CTRL2_G   0x11
#define LSM6DS3_OUTX_L_XL 0x28
#define OUTX_L_G 0x22

#define PIN_OCTO_1 4
#define PIN_OCTO_2 5
#define PIN_OCTO_3 6
#define PIN_OCTO_4 7
#define PIN_OCTO_5 8
#define PIN_OCTO_6 9
#define PIN_OCTO_7 10
#define PIN_OCTO_8 11
#define PIN_OCTO_9 12
#define PIN_OCTO_10 13
#define PIN_OCTO_11 14
#define PIN_OCTO_12 15

/////////////////////////////////////////////////////
//VARIABLES GLOBALES

//Timer
uint64_t start_time = 0, end_time = 0;
double elapsed_time_us = 0;

uint64_t periodo_imu = 0;
uint64_t periodo_sonar = 0;

//Medicion
double medicion = 0;
double CORRECCION_IMU_ACEL = (double)100/(double)170408;
double CORRECCION_IMU_GIRO = (double)4.375/(double)1000 * (double)0.0175;
int16_t accel_data[3];
int16_t giro_data[3];

//Variables nodo
rcl_node_t node;

rcl_publisher_t publisher_sonar;
rcl_publisher_t publisher_imu;

rcl_subscription_t subscriber_cant_sonar;
rcl_subscription_t subscriber_seguridad;

sensor_msgs__msg__Range__Sequence* sonar_msg;
sensor_msgs__msg__Imu imu_msg;

rcl_timer_t timer_publ_sonar;
rcl_timer_t timer_publ_imu;
int64_t periodo;

rclc_support_t support;

rcl_allocator_t allocator;

// Message object to receive publisher data
std_msgs__msg__Int16 msg_sensores;
std_msgs__msg__Float32 msg_seg;

//Multiplexado
int n_sensor_leido = 0;
int lugar_sensor_leido = 0;

bool medicion_nueva = 0;

//Definiciones
bool sensores_activar[12] = {1,1,1,1,1,1,1,1,1,0,0,0};
int cant_sensores = 0;
int frec_imu = FREC_IMU;
int new_frec_imu = FREC_IMU;
int frec_sonar = FREC_SONAR;

//Cuenta para publicacion
int contador_imu = 0;
int contador_sonar = 0;
int contador_frec = 0;

//Sensado distancia minima
double umbral_seguridad = UMBRAL_SEGURIDAD;
int sensor_seguridad = 0;

/////////////////////////////////////////////////////
//FUNCIONES

/*! \brief Inicializar todos los pines de GPIO
 */
void init_pins(){

    //Seteo de pines
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(LED_INTEGRADO);
    gpio_set_dir(LED_INTEGRADO, GPIO_OUT);

    gpio_init(ECHO);
    gpio_set_dir(ECHO, GPIO_IN);

    gpio_init(INIT);
    gpio_set_dir(INIT, GPIO_OUT);

    gpio_init(PIN_OCTO_12);
    gpio_set_dir(PIN_OCTO_12, GPIO_OUT);
    gpio_init(PIN_OCTO_11);
    gpio_set_dir(PIN_OCTO_11, GPIO_OUT);
    gpio_init(PIN_OCTO_10);
    gpio_set_dir(PIN_OCTO_10, GPIO_OUT);
    gpio_init(PIN_OCTO_9);
    gpio_set_dir(PIN_OCTO_9, GPIO_OUT);
    gpio_init(PIN_OCTO_8);
    gpio_set_dir(PIN_OCTO_8, GPIO_OUT);
    gpio_init(PIN_OCTO_7);
    gpio_set_dir(PIN_OCTO_7, GPIO_OUT);
    gpio_init(PIN_OCTO_6);
    gpio_set_dir(PIN_OCTO_6, GPIO_OUT);
    gpio_init(PIN_OCTO_5);
    gpio_set_dir(PIN_OCTO_5, GPIO_OUT);
    gpio_init(PIN_OCTO_4);
    gpio_set_dir(PIN_OCTO_4, GPIO_OUT);
    gpio_init(PIN_OCTO_3);
    gpio_set_dir(PIN_OCTO_3, GPIO_OUT);
    gpio_init(PIN_OCTO_2);
    gpio_set_dir(PIN_OCTO_2, GPIO_OUT);
    gpio_init(PIN_OCTO_1);
    gpio_set_dir(PIN_OCTO_1, GPIO_OUT);

    gpio_init(SEG_1);
    gpio_set_dir(SEG_1, GPIO_OUT);
    gpio_init(SEG_2);
    gpio_set_dir(SEG_2, GPIO_OUT);
}

/*! \brief setear UART para la comunicacion serial de ROS
 */
void init_uart(){

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    uart_init(UART_USADA, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);    
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

}

/*! \brief Inicializar I2C para la comunicacion con la IMU
 */
void init_i2c(){

    //INICIALIZAR I2C 
    i2c_init(I2C_PORT, FREQ_I2C);

    //Setear pines SDA SCL
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

}


/*! \brief Inicializar arreglo de msg de sonar (sensor_msgs__msg__Range)
 */
void init_sonar(){

    sonar_msg = sensor_msgs__msg__Range__Sequence__create(MAX_SENSORES);
    sensor_msgs__msg__Range__Sequence__init(sonar_msg,MAX_SENSORES);

    //Definir que cantidad de sensores estaran activos en un principio
    int nueva_cantidad;

    for(int i = 0; i < MAX_SENSORES; i++){

        if(sensores_activar[i]){

            nueva_cantidad = nueva_cantidad+1;

        }
        
    }

    cant_sensores = nueva_cantidad;

    char frame_id[20];
    for (int i = 1; i <= MAX_SENSORES; i++) {

        // Construir el nombre del sensor con el número correspondiente
        snprintf(frame_id, sizeof(frame_id), "sensor_%d", i);

        // Asignar el nombre construido a la estructura
        strcpy(sonar_msg->data[i-1].header.frame_id.data, frame_id);
        sonar_msg->data[i-1].header.frame_id.size = strlen(sonar_msg->data[i-1].header.frame_id.data);

        sonar_msg->data[i-1].field_of_view = FIELD_VIEW;

        sonar_msg->data[i-1].radiation_type = 0;

        sonar_msg->data[i-1].max_range = MAX_DIST;

        sonar_msg->data[i-1].min_range = MIN_DIST;

    }

}


/*! \brief Inicializar mensaje IMU (sensor_msgs__msg__Imu)
 */
void init_imu(){

    memset(&imu_msg, 0, sizeof(sensor_msgs__msg__Imu));
    sensor_msgs__msg__Imu__init(&imu_msg);

    strcpy(imu_msg.header.frame_id.data, "IMU LSM6DS3");

}


/*! \brief Funcion llamada al reasignar cantidad de sensores
 * 
 * \param msgin mensaje entrante
 */
void subscription_callback_sensores(const void * msgin)
{

    // Cast received message to used type
    const std_msgs__msg__Int16 * msg_sonar = (const std_msgs__msg__Int16 *)msgin;

    // Variable to hold the processed value
    uint16_t recibido = (uint16_t) msg_sonar->data;  // Cast and get value directly
    int nueva_cantidad = 0;

    // Crear el arreglo bool (suponiendo que sensores_activar ya está definido en otro lugar)
    for (int i = 0; i < MAX_SENSORES; i++) {

        // Extraer el bit correspondiente y almacenar en el arreglo
        sensores_activar[i] = (recibido & (1 << i)) != 0;

    }

    for(int i = 0; i < MAX_SENSORES; i++){

        if(sensores_activar[i]){

            nueva_cantidad++;

        }
        
    }

    cant_sensores = nueva_cantidad;
    
    n_sensor_leido = 0;
    lugar_sensor_leido = 0;

    actualizar_periodo_sonar();

    gpio_put(SEG_1,0);
    gpio_put(SEG_2,0);

}


/*! \brief Funcion llamada para reasignar el periodo del sonar al cambiar la cantidad de sensores activos
 */
void actualizar_periodo_sonar(){

    //VER QUE CANTIDAD DE SENSORES ACTIVAR
    int nueva_cantidad;

    for(int i = 0; i < MAX_SENSORES; i++){

        if(sensores_activar[i]){

            nueva_cantidad = nueva_cantidad+1;

        }
        
    }

    cant_sensores = nueva_cantidad;

    rcl_timer_exchange_period(&timer_publ_sonar, RCL_S_TO_NS((double)1/(frec_sonar*cant_sensores)),&periodo_sonar);

}


/*! \brief Funcion llamada al reasignar la distancia de seguridad
 * 
 * \param msgin mensaje entrante
 */
void subscription_callback_seguridad(const void * msgin)
{

    float nueva_dist = 0;

    // Cast received message to used type
    const std_msgs__msg__Float32 * msg_imu = (const std_msgs__msg__Float32 *)msgin;

    // Variable to hold the processed value
    nueva_dist = (int) msg_imu->data;  // Cast and get value directly

    nueva_dist = nueva_dist;

    if(nueva_dist <= MAX_DIST && nueva_dist >= MIN_DIST){

        umbral_seguridad = nueva_dist;

        gpio_put(SEG_1,0);
        gpio_put(SEG_2,0);

    }

}

/*! \brief Leer y guardar la medicion de la IMU
 * 
 * \param accel_data datos del acelerometro
 * \param giro_data datos del giroscopio
 */
void LSM6DS3_Read_Save(int16_t *accel_data, int16_t *giro_data)
{
    uint8_t data[6];
    uint8_t reg;

    // Read accelerometer data
    reg = LSM6DS3_OUTX_L_XL;  
    i2c_write_blocking(I2C_PORT, LSM6DS3_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, LSM6DS3_ADDR, data, 6, false);

    accel_data[0] = (int16_t)(data[1] << 8 | data[0]);  // X-axis
    accel_data[1] = (int16_t)(data[3] << 8 | data[2]);  // Y-axis
    accel_data[2] = (int16_t)(data[5] << 8 | data[4]);  // Z-axis

    // Scale the accelerometer data
    imu_msg.linear_acceleration.x = accel_data[0] * CORRECCION_IMU_ACEL;
    imu_msg.linear_acceleration.y = accel_data[1] * CORRECCION_IMU_ACEL;
    imu_msg.linear_acceleration.z = accel_data[2] * CORRECCION_IMU_ACEL;

    // Read gyroscope data
    reg = OUTX_L_G;  // Changed to the correct gyroscope data register
    i2c_write_blocking(I2C_PORT, LSM6DS3_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, LSM6DS3_ADDR, data, 6, false);

    giro_data[0] = (int16_t)(data[1] << 8 | data[0]);  // X-axis
    giro_data[1] = (int16_t)(data[3] << 8 | data[2]);  // Y-axis
    giro_data[2] = (int16_t)(data[5] << 8 | data[4]);  // Z-axis

    imu_msg.angular_velocity.x = giro_data[0] * CORRECCION_IMU_GIRO;
    imu_msg.angular_velocity.y = giro_data[1] * CORRECCION_IMU_GIRO;
    imu_msg.angular_velocity.z = giro_data[2] * CORRECCION_IMU_GIRO;

    // Timestamp for the message
    imu_msg.header.stamp.nanosec = time_us_64() % 1000000;
    imu_msg.header.stamp.sec = time_us_64() / 1000000;
}

/*! \brief Funcion de error. 
 * 
 * \brief Se entra en un bucle infinito mientras el LED de error parpadea.
 * 
 * \param error Codigo de error a mostrar
 */
void error_func(int error)
{
    while (1) {

        for(int i = 0; i < error; i++){

            gpio_put(LED_PIN, 1);
            sleep_ms(500);  
            gpio_put(LED_PIN, 0);
            sleep_ms(500); 

        }

        gpio_put(LED_PIN, 0);
        sleep_ms(3000);  
    
    }
}

void publicar_imu(rcl_timer_t *timer, int64_t last_call_time){

    LSM6DS3_Read_Save(accel_data, giro_data); // Actualiza los datos del IMU

    rcl_publish(&publisher_imu, &imu_msg, NULL);

}


/*! \brief Funcion llamada por interrupcion de timer que envia mensaje de ROS
 * 
 * \param timer timer asociado
 * \param last_call_time tiempo de ultimo llamado
 */
void publicar_sonar(rcl_timer_t *timer, int64_t last_call_time)
{

    //Timeout para el caso que se desconecte un sensor
    if(medicion_nueva == 0){

        sonar_msg->data[lugar_sensor_leido].header.stamp.nanosec=0;
        sonar_msg->data[lugar_sensor_leido].header.stamp.sec=0;
        sonar_msg->data[lugar_sensor_leido].range=MAX_DIST;

    }

    rcl_ret_t ret = rcl_publish(&publisher_sonar, &sonar_msg->data[lugar_sensor_leido], NULL);

    //sonar_publicado ++;
    lugar_sensor_leido ++;

    cambiar_sensor();

    medicion_nueva = 0;

}


/*! \brief Multiplexar para leer el siguiente sensor
 */
void cambiar_sensor(){

   if(cant_sensores == 0){

        lugar_sensor_leido = 12;

   }else{ 

        while(sensores_activar[lugar_sensor_leido] == 0){

            if(lugar_sensor_leido >= MAX_SENSORES){

                lugar_sensor_leido = 0;

            }else{

                lugar_sensor_leido++;

            }

        }

   }

    if (n_sensor_leido < cant_sensores-1){

        n_sensor_leido++;

    }else{

        n_sensor_leido = 0;

    }



    switch (lugar_sensor_leido)
    {
    case 0:
        gpio_put(PIN_OCTO_12,1);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,0);
        break;

    case 1:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,1);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,0);
        break;

    case 2:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,1);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,0);
        break;

    case 3:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,1);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,0);
        break;
    
    case 4:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,1);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,0);
        break;

    case 5:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,1);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,0);
        break;

    case 6:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,1);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,0);
        break;

    case 7:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,1);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,0);
        break;

    case 8:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,1);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,0);
        break;

    case 9:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,1);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,0);
        break;

    case 10:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,1);
        gpio_put(PIN_OCTO_1,0);
        break;

    case 11:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,1);
        break;

    case 12:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,0);
        break;


    default:
        gpio_put(PIN_OCTO_12,0);
        gpio_put(PIN_OCTO_11,0);
        gpio_put(PIN_OCTO_10,0);
        gpio_put(PIN_OCTO_9,0);
        gpio_put(PIN_OCTO_8,0);
        gpio_put(PIN_OCTO_7,0);
        gpio_put(PIN_OCTO_6,0);
        gpio_put(PIN_OCTO_5,0);
        gpio_put(PIN_OCTO_4,0);
        gpio_put(PIN_OCTO_3,0);
        gpio_put(PIN_OCTO_2,0);
        gpio_put(PIN_OCTO_1,0);
        break;
    }

}

/*! \brief Medicion de distancia por interrupcion en el pin de ECHO
 * 
 * \param gpio pin asociado
 * \param events evento ocurrido
 */  
void callback_echo(uint gpio, uint32_t events) {

    //Flag de nueva medicion
    medicion_nueva = 1;

    // Interrupción por flanco ascendente
    if (events & GPIO_IRQ_EDGE_RISE) {

        end_time = time_us_64();

        elapsed_time_us = (end_time - start_time);

        medicion = ((elapsed_time_us*328)/20000)/100;

    } 

    if (medicion>MAX_DIST){

        medicion = MAX_DIST;

    }

    sonar_msg->data[lugar_sensor_leido].header.stamp.nanosec=end_time%1000000;
    sonar_msg->data[lugar_sensor_leido].header.stamp.sec=end_time/1000000;
    sonar_msg->data[lugar_sensor_leido].range=medicion;


    //CONTROLAR QUE MEDICION NO ESTE POR DEBAJO DE UMBRAL
    if(medicion < umbral_seguridad){

        gpio_put(SEG_1,1);
        gpio_put(SEG_2,1);

        sensor_seguridad = lugar_sensor_leido;

    }
    
    if(lugar_sensor_leido == sensor_seguridad && medicion + HISTERESIS > umbral_seguridad){

        gpio_put(SEG_1,0);
        gpio_put(SEG_2,0);

    }

}


/*! \brief Inicializar IMU
 */  
void LSM6DS3_Init()
{
    uint8_t config[2];

    // Configuración del acelerómetro (CTRL1_XL)
    config[0] = LSM6DS3_CTRL1_XL;
    config[1] = 0x60; // Acelerómetro a 416 Hz, ±2g, modo normal

    i2c_write_blocking(I2C_PORT, LSM6DS3_ADDR, config, 2, false);

    // Configuración del giroscopio (CTRL2_G)
    config[0] = LSM6DS3_CTRL2_G;
    config[1] = 0x60; // Giroscopio a 416 Hz, 245 dps, modo normal

    i2c_write_blocking(I2C_PORT, LSM6DS3_ADDR, config, 2, false);
}

/////////////////////////////////////////////////////
//MAIN
////////////////////////////////////////////////////

int main(){

    /////////////////////////////////////////////////////
    //INICIALIZACION GPIO / UART / I2C / LSM6DS3

    stdio_init_all();

    init_pins();

    /////////////////////
    //LED DE SET UP (SE APAGA AL TERMINAR INICIALIZACION)
    gpio_put(LED_PIN, 1);
    /////////////////////

    init_uart();

    init_i2c();

    sleep_ms(500);

    LSM6DS3_Init();

    /////////////////////////////////////////////////////
    //CONFIGURACION ROS

    allocator = rcl_get_default_allocator();

    // Inicializar support object
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);

    //Esperar a conexion con maquina a bordo
    rcl_ret_t ret = rmw_uros_ping_agent(TIMEOUT_MS, INTENTOS_CONN);
    if (ret != RCL_RET_OK) {
        error_func(1);
    }

    // Inicializar nodo
    const char * node_name = NODE_NAME;

    const char * namespace = NAMESPACE_NAME;

    rc = rclc_node_init_default(&node, node_name, namespace, &support);
    if (rc != RCL_RET_OK) {
        error_func(2);    
    }

    // Init topico range
    rc = rclc_publisher_init_default(
        &publisher_sonar,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        TOPIC_NAME_SONAR);
    if (rc != RCL_RET_OK) {
        error_func(3);    
    }

    // Init topico imu
    rc = rclc_publisher_init_default(
        &publisher_imu,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        TOPIC_NAME_IMU);
    if (rc != RCL_RET_OK) {
        error_func(4);    
    }

    // Init suscriptor CANT SONAR
    rc = rclc_subscription_init_default(
        &subscriber_cant_sonar, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
        TOPIC_NAME_CANT_SENSORES);
    if (RCL_RET_OK != rc) {
        error_func(5);
    } 

    // Init suscriptor SEGURIDAD
    rc = rclc_subscription_init_default(
        &subscriber_seguridad, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), 
        TOPIC_NAME_DIST_SEG);
    if (RCL_RET_OK != rc) {
        error_func(6);
    } 

    //////////////////////////////////////////////////////
    //INICIALIZACION DATOS MENSAJES

    //Inicializacion array sonar
    init_sonar();

    //Inicializacion imu
    init_imu();

    //Inicializacion suscribers

    memset(&msg_sensores, 0, sizeof(std_msgs__msg__Int16));
    std_msgs__msg__Int16__init(&msg_sensores);

    memset(&msg_seg, 0, sizeof(std_msgs__msg__Float32));
    std_msgs__msg__Float32__init(&msg_seg);

    rclc_executor_t executor_pub_sonar;
    rclc_executor_init(&executor_pub_sonar, &support.context, 1, &allocator);

    rclc_executor_t executor_pub_imu;
    rclc_executor_init(&executor_pub_imu, &support.context, 1, &allocator);

    rclc_executor_t executor_sub_cant;
    rclc_executor_init(&executor_sub_cant, &support.context, 1, &allocator); 

    rclc_executor_t executor_sub_seg;
    rclc_executor_init(&executor_sub_seg, &support.context, 1, &allocator);

    //////////////////////////////////////////////////////
    //INTERRUPCIONES ROS

    periodo_imu = RCL_S_TO_NS((double)1/FREC_IMU);
    periodo_sonar =  RCL_S_TO_NS((double)1/FREC_IMU);

    //Interrupcion por timer para publicacion
    rclc_timer_init_default(&timer_publ_sonar,&support,periodo_sonar,&publicar_sonar);

    rclc_timer_init_default(&timer_publ_imu,&support,periodo_imu,&publicar_imu);

    rclc_executor_add_subscription(&executor_sub_cant, &subscriber_cant_sonar, &msg_sensores, &subscription_callback_sensores, ON_NEW_DATA);

    rclc_executor_add_subscription(&executor_sub_seg, &subscriber_seguridad, &msg_seg, &subscription_callback_seguridad, ON_NEW_DATA);

    //////////////////////////////////////////////////////

    //Vincular timer
    rclc_executor_add_timer(&executor_pub_sonar, &timer_publ_sonar);

    rclc_executor_add_timer(&executor_pub_imu, &timer_publ_imu);

    //////////////////////////////////////////////////////
    //INTERRUPCION ECHO SONAR

    //Interrupcion gpio ECHO
    gpio_set_irq_enabled_with_callback(ECHO, GPIO_IRQ_EDGE_RISE, true, &callback_echo);

    actualizar_periodo_sonar();

    /////////////////////
    //FINALIZACION SET UP
    gpio_put(LED_PIN, 0);
    /////////////////////

    /////////////////////////////////////////////////////
    //EJECUCION INFINITA

    while(1){
        
        //Chequear por datos nuevos
        rclc_executor_spin_some(&executor_pub_sonar, RCL_MS_TO_NS(10));
        rclc_executor_spin_some(&executor_pub_imu, RCL_MS_TO_NS(10));
        rclc_executor_spin_some(&executor_sub_cant, RCL_MS_TO_NS(10));
        rclc_executor_spin_some(&executor_sub_seg, RCL_MS_TO_NS(10));


        //Si la ultima medicion ya se envio, iniciar una medicion nueva.
        if(medicion_nueva == 0){

            //INICIAR NUEVA MEDICION
            start_time = time_us_64();
            gpio_put(INIT, 1);
            sleep_ms(TON_INIT);  
            gpio_put(INIT, 0);
            sleep_ms(TOFF_INIT);

        }

    }

    return 0;

}