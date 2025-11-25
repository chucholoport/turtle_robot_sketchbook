/*
 * Proyecto: Blink controlado por ROS
 * Autor: Jesús López
 * Fecha: 25/11/2025
 *
 * Descripción:
 * Este sketch convierte un Arduino en un nodo ROS usando rosserial.
 * El LED conectado al pin 13 se enciende o apaga según los mensajes
 * publicados en el tópico /toggle_led.
 *
 * Entorno:
 * - Ubuntu 20.04
 * - ROS Noetic
 * - Paquete rosserial_arduino generado con make_libraries.py
 *
 * Flujo ROS:
 * 1. Un nodo en ROS publica mensajes std_msgs/Bool en el tópico /toggle_led.
 * 2. El puente rosserial (rosrun rosserial_python serial_node.py /dev/ttyUSB0)
 *    transmite esos mensajes al Arduino por USB.
 * 3. El Arduino, como subscriber, recibe el mensaje y enciende/apaga el LED.
 *
 * Ejemplo de publicación desde ROS:
 *   rostopic pub /toggle_led std_msgs/Bool "data: true"
 *   rostopic pub /toggle_led std_msgs/Bool "data: false"
 *
 * Objetivo didáctico:
 * - Mostrar cómo un microcontrolador puede integrarse como nodo ROS.
 * - Practicar la comunicación publisher → subscriber entre ROS y hardware real.
 */

#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

int ledPin = 13;

void messageCb(const std_msgs::Bool& toggle_msg) {
  if (toggle_msg.data) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}

ros::Subscriber<std_msgs::Bool> sub("toggle_led", &messageCb);

void setup() {
  pinMode(ledPin, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
