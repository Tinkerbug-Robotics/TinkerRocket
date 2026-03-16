const int PYRO_1_ARM = 14;
const int PYRO_1_CONT = 16;
const int PYRO_1_FIRE = 15;

const int PYRO_2_ARM = 22;
const int PYRO_2_CONT = 19;
const int PYRO_2_FIRE = 18;

const int IND1_PIN = 29; //22;
const int IND2_PIN = 2; //23;

void setup()
{

    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting...");

    pinMode(IND1_PIN, OUTPUT);
    pinMode(IND2_PIN, OUTPUT);

    digitalWrite(IND1_PIN, HIGH);
    digitalWrite(IND2_PIN, HIGH);
    delay(1500);
    digitalWrite(IND1_PIN, LOW);
    digitalWrite(IND2_PIN, LOW);

    pinMode(PYRO_1_ARM, OUTPUT);
    digitalWrite(PYRO_1_ARM, LOW);

    // Immediatly turn off pyro fire
    pinMode(PYRO_1_FIRE, OUTPUT);
    digitalWrite(PYRO_1_FIRE, LOW);

    // Continuity check is an input
    pinMode(PYRO_1_CONT, INPUT);

    pinMode(PYRO_2_ARM, OUTPUT);
    digitalWrite(PYRO_2_ARM, LOW);

    // Immediatly turn off pyro fire
    pinMode(PYRO_2_FIRE, OUTPUT);
    digitalWrite(PYRO_2_FIRE, LOW);

    // Continuity check is an input
    pinMode(PYRO_2_CONT, INPUT);

    digitalWrite(PYRO_1_ARM, HIGH);

}

void loop()
{

    Serial.println("--------------");
    Serial.println("Fire");
    digitalWrite(PYRO_1_FIRE, HIGH);
    digitalWrite(PYRO_2_FIRE, HIGH);
    delay(2000);
    Serial.println("Stop Firing");
    digitalWrite(PYRO_1_FIRE, LOW);
    digitalWrite(PYRO_2_FIRE, LOW);
    delay(2000);
    

    // // Sequence
    // Serial.println("--------------------");
    // digitalWrite(IND2_PIN, HIGH);
    // digitalWrite(IND1_PIN, HIGH);
    // delay(500);
    // digitalWrite(IND2_PIN, LOW);
    // digitalWrite(IND1_PIN, LOW);
    // delay(500);
    // Serial.println("Pyro 2 SAFE");
    // digitalWrite(PYRO_2_ARM, LOW);
    // delay(100);
    // Serial.print("Continuity: ");Serial.println(digitalRead(PYRO_2_CONT));
    // if (digitalRead(PYRO_2_CONT))
    //     digitalWrite(IND2_PIN, HIGH);
    // else
    //     digitalWrite(IND2_PIN, LOW);
    // delay(2000);
    // Serial.println("Pyro 2 ARM");
    // digitalWrite(IND2_PIN, HIGH);
    // digitalWrite(PYRO_2_ARM, HIGH);
    // Serial.print("Continuity: ");Serial.println(digitalRead(PYRO_1_CONT));
    // if (digitalRead(PYRO_2_CONT))
    //     digitalWrite(IND2_PIN, HIGH);
    // else
    //     digitalWrite(IND2_PIN, LOW);
    // delay(2000);
    // digitalWrite(PYRO_2_FIRE, HIGH);
    // Serial.println("Pyro 2 FIRE");
    // delay(1000);
    // digitalWrite(PYRO_2_FIRE, LOW);
    // digitalWrite(PYRO_2_ARM, LOW);
    // digitalWrite(IND2_PIN, LOW);

    // delay(2000);

    // Serial.println("Pyro 2 SAFE");
    // digitalWrite(PYRO_2_ARM, LOW);
    // delay(100);
    // Serial.print("Continuity: ");Serial.println(digitalRead(PYRO_2_CONT));
    // delay(100);
    // Serial.println("Pyro 2 ARM");
    // digitalWrite(PYRO_2_ARM, HIGH);
    // Serial.print("Continuity: ");Serial.println(digitalRead(PYRO_2_CONT));
    // delay(1000);
    // digitalWrite(PYRO_2_FIRE, HIGH);
    // Serial.println("Pyro 2 FIRE");
    // delay(1000);
    // digitalWrite(PYRO_2_FIRE, LOW);
    // digitalWrite(PYRO_2_ARM, LOW);
    delay(2000);


}
