#include "legged_common/GamePad.hpp"

void GamePad::intiSDL2() {
    const char* mapping = "050000004c050000cc09000001000000,Sony DualShock 4,a:b1,b:b2,back:b8,dpdown:h0.4,dpleft:h0.8,dpright:h0.2,dpup:h0.1,guide:b12,leftshoulder:b4,leftstick:b10,lefttrigger:a3,leftx:a0,lefty:a1,rightshoulder:b5,rightstick:b11,righttrigger:a4,rightx:a2,righty:a5,start:b9,touchpad:b13,x:b0,y:b3,platform:Linux,";
    // const char* mapping = "030000004c050000c405000000000000,PS4 Controller,a:b0,b:b1,back:b8,dpdown:h0.4,dpleft:h0.8,dpright:h0.2,dpup:h0.1,guide:b10,leftshoulder:b4,leftstick:b11,lefttrigger:a2,rightshoulder:b5,rightstick:b12,righttrigger:a5,start:b9,x:b2,y:b3,leftx:a0,lefty:a1,rightx:a3,righty:a4";

    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK | SDL_INIT_EVENTS) < 0) {
        std::cerr << "SDL could not initiated: " << SDL_GetError() << std::endl;
    }
    SDL_GameControllerAddMapping(mapping);

    // Try to find and open a gamepad
    bool gamepad_found = false;
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            controller = SDL_GameControllerOpen(i);
            if (controller) {
                std::cout << "Gamepad routines enabled: " << SDL_GameControllerName(controller) << std::endl;
                gamepad_found = true;
                break;
            } else {
                std::cerr << "Gamepad routines could not be enabled: " << SDL_GetError() << std::endl;
            }
        }
    }
    if (!gamepad_found) {
        std::cout << "Gamepad not found - it will be reconnected when connected" << std::endl;
    }

}


void GamePad::gamepad() {
    switch (event.type) {
        
        // 1. GAMEPAD BAĞLANTI/AYIRMA OLAYLARI (ALLEGRO_EVENT_JOYSTICK_CONFIGURATION yerine)
        case SDL_CONTROLLERDEVICEADDED:
            // event.cdevice.which is the joystick device index
            if (!controller) {
                if (SDL_IsGameController(event.cdevice.which)) {
                    controller = SDL_GameControllerOpen(event.cdevice.which);
                    if (controller) {
                        std::cout << "Gamepad connected: " << SDL_GameControllerName(controller) << std::endl;
                    } else {
                        std::cerr << "Failed to open gamepad at index " << event.cdevice.which 
                                  << ": " << SDL_GetError() << std::endl;
                    }
                }
            }
            break;
            
        case SDL_CONTROLLERDEVICEREMOVED:
            if (controller && event.cdevice.which == SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controller))) {
                SDL_GameControllerClose(controller);
                controller = nullptr;
                std::cout << "Gamepad disconnected." << std::endl;
            }
            break;

        // 2. ANALOG ÇUBUK VE TETİK EKSEN HAREKETLERİ (ALLEGRO_EVENT_JOYSTICK_AXIS yerine)
        case SDL_CONTROLLERAXISMOTION:
        {
            // SDL eksen değeri -32768 ile 32767 arasındadır.
            // Değeri -1.0 ile 1.0 arasına normalize etmeliyiz.
            float pos = (float)event.caxis.value / 32767.0f;
            
            // Tolerans değeri (Deadzone). Allegro'da 0.03 eşiği kullanmışsınız.
            if (std::abs(pos) < 0.05) { 
                pos = 0.0f; 
            }

            // Allegro'da kullanılan ev.joystick.pos değeri yaklaşık -1.0 ile 1.0 arasındaydı.
            // Bu nedenle, Allegro'daki kod mantığınızı korumak için normalize edilmiş 'pos' değerini kullanacağız.
            float normalized_l2 = 1.0f - 2.0f * ((float)event.caxis.value / 32767.0f);                     
            float normalized_r2 = 1.0f - 2.0f * ((float)event.caxis.value / 32767.0f);                  

            switch ((SDL_GameControllerAxis)event.caxis.axis) {
                
                // --- SOL ÇUBUK ---
                case SDL_CONTROLLER_AXIS_LEFTY: // Y-ekseni (Allegro: stick 0, axis 0)
                    if(standUp && walkEnable) {
                        Vx_mean = -Kv * pos; 
                        Vx_mean = std::floor(Vx_mean * 10000.0) / 10000.0;
                        if (std::abs(Vx_mean) < 0.03) { Vx_mean = 0.0; }
                        joyCmd[0] = Vx_mean;
                    } else {
                        joyCmd[0] = 0.0;
                    }
                    break;
                    
                case SDL_CONTROLLER_AXIS_LEFTX: // X-ekseni (Allegro: stick 0, axis 1)
                    if(standUp && walkEnable) {
                    Vy_mean = -0.3 * pos;
                        Vy_mean = std::floor(Vy_mean * 10000.0) / 10000.0;
                        if (std::abs(Vy_mean) < 0.03) { Vy_mean = 0.0; }
                        joyCmd[1] = Vy_mean;
                    } else {
                        joyCmd[1] = 0.0;
                    }
                    break;

                // --- SAĞ ÇUBUK ---
                case SDL_CONTROLLER_AXIS_RIGHTY: // Y-ekseni (Allegro: stick 1, axis 1)
                    if(standUp && !walkEnable) {
                        pitch = 0.2 * pos;
                        if (std::abs(pitch) < 0.03) { pitch = 0.0; }
                    } else pitch = 0.0;
                    joyCmd[3] = pitch;
                    break;

                case SDL_CONTROLLER_AXIS_RIGHTX: // X-ekseni (Allegro: stick 2, axis 0)
                    if(standUp) {
                        if (walkEnable) { 
                            Vyaw = -pos;
                            joyCmd[2] = Vyaw;
                        } else {
                            yaw = -0.35 * pos; 
                            joyCmd[2] = yaw; 
                        }
                        if (std::abs(joyCmd[2]) < 0.03) { joyCmd[2] = 0.0; }
                    } else joyCmd[2] = 0.0;
                    break;

                // --- TETİKLER (L2/R2) ---
                // SDL'de tetikler [0, 32767] aralığındadır. Bu nedenle normalize edilmesi farklıdır.
                case SDL_CONTROLLER_AXIS_TRIGGERLEFT: // L2 (Allegro: stick 1, axis 0)
                    // pos, [0, 1.0] aralığındadır. 
                    // Allegro'daki ev.joystick.pos genellikle [-1.0, 1.0] arasındadır, 
                    // ancak tetikler genellikle sadece pozitif alana sahiptir. 
                    // Allegro'daki haritalama mantığınıza uygun bir dönüşüm yapın.
                    // Varsayım: Allegro'daki L2 ekseni [1.0, -1.0] aralığında çalışıyordu.
                    
                    // L2 için 0 -> 1.0 (çekilmedi) ve 32767 -> -1.0 (tam çekildi) dönüşümünü taklit ediyoruz:
                    if(standUp) {
                        cmdZc = mapVal(normalized_l2, 1.0, -1.0, 0.3, MIN_BODY_HEIGHT);
                        joyCmd[5] = cmdZc;
                    } 
                    else joyCmd[5] = joyCmd[5];
                    break;

                case SDL_CONTROLLER_AXIS_TRIGGERRIGHT: // R2 (Allegro: stick 2, axis 1)
                    // R2 için [1.0, -1.0] aralığını taklit ediyoruz:
                    if(standUp) {
                        cmdZc = mapVal(normalized_r2, 1.0, -1.0, 0.3, MAX_BODY_HEIGHT);
                        joyCmd[5] = cmdZc;
                    } 
                    else joyCmd[5] = joyCmd[5];
                    joyCmd[5] = cmdZc;
                    break;
                    
                default:
                    break;
            }
            break;
        }

        // 3. DÜĞME BASILMA OLAYLARI (ALLEGRO_EVENT_JOYSTICK_BUTTON_DOWN yerine)
        case SDL_CONTROLLERBUTTONDOWN:
        {
            switch ((SDL_GameControllerButton)event.cbutton.button) {
                
                case SDL_CONTROLLER_BUTTON_BACK: // SHARE butonu (Allegro: case 8)
                    close = true;
                    // Al_release_joystick yerine SDL'de açık olan controller'ı kapatın.
                    // SDL_GameControllerClose(controller);
                    break;
                    
                case SDL_CONTROLLER_BUTTON_START: // OPTIONS butonu (Allegro: case 9)
                    walkEnable = !walkEnable;                    
                    std::cout << (walkEnable ? "Walking mode enabled!" : "Walking mode disabled!") << std::endl;
                    break;

                case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
                    selectedGait = selectedGait - 1;                    
                    if(selectedGait < 1) selectedGait = 3;
                    std::cout << "Gait set to: " << selectedGait << std::endl;
                    break;

                case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
                    selectedGait = selectedGait + 1;
                    if(selectedGait > 3) selectedGait = 1;
                    std::cout << "Gait set to: " << selectedGait << std::endl;
                    break;
                    
                // D-Pad Hız Kontrolü (Allegro'da AXIS olarak işlenmişti, SDL'de BUTTON olarak)
                case SDL_CONTROLLER_BUTTON_DPAD_UP:
                    Kv = Kv + 0.1;
                    if(Kv >= 2.0) { Kv = 2.0;}
                    std::cout << "Max. Vel. set to: " << Kv << std::endl;
                    break;
                    
                case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
                    Kv = Kv - 0.1;
                    if(Kv <= 0.1) { Kv = 0.1; }
                    std::cout << "Max. Vel. set to: " << Kv << std::endl;
                    break;
                
                case SDL_CONTROLLER_BUTTON_X: // Square button on PS controllers
                    cmdZc = 0.3;
                    joyCmd[5] = cmdZc;
                    standUp = true;
                    std::cout << "Height is set to 0.3" << std::endl;
                    break;
                
                case SDL_CONTROLLER_BUTTON_B: // Circle button on PS controllers
                    cmdZc = 0.0369;
                    joyCmd[5] = cmdZc;
                    standUp = false;
                    std::cout << "Height is set to 0.0369" << std::endl;
                    break;
                    
                default:
                    break;
            }
            break;
        }

        // 4. DÜĞME BIRAKILMA OLAYLARI (ALLEGRO_EVENT_JOYSTICK_BUTTON_UP yerine)
        case SDL_CONTROLLERBUTTONUP:
            // İhtiyaç duyulan düğme bırakılma mantığı buraya eklenebilir.
            break;

        default:
            // Diğer olayları yönet
            break;
    }
    if(walkEnable) gait = selectedGait;
    else gait = 0;
}

void GamePad::callGamePad() {
    // Process all pending SDL events first (this is important for device addition events)
    while(SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            close = true;
        } else {
            gamepad();
        }
    }
    
    // If no controller is connected after processing events, try to find one
    // This handles the case where a controller was connected but the event wasn't processed
    if (!controller) {
        for (int i = 0; i < SDL_NumJoysticks(); ++i) {
            if (SDL_IsGameController(i)) {
                controller = SDL_GameControllerOpen(i);
                if (controller) {
                    std::cout << "Gamepad reconnected: " << SDL_GameControllerName(controller) << std::endl;
                    break;
                }
            }
        }
    }
}

double GamePad::mapVal(double inVal, double inMax, double inMin, double outMax, double outMin) { 
    double mappedVal = (inVal - inMin)*((outMax-outMin)/(inMax - inMin)) + outMin;
    return mappedVal;
}