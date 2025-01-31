Hall effekt ström sensor för att mäta inspänningen till varje fas för att kunna räkna ut hastigheten på motorn.
Skapa ett PID system genom att skicka en desired speed till ESP32, mät sedan strömmen ut från ESCn som sedan går in till en arduino som kör PID reglerar strömmen.

Vi kan börja med att skapa ett system, och så länge vi kör samma motorer och ESCs kommer det att fungera för varje motor.
