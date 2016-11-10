function keyboardEventListener(~,event)
    %keyboardEventListener Invoked when a keyboard character is pressed.
    global keypressFrame;
    global keypressDataReady;
    global keypressKey;
    global keypressKeyPrev;
    
    keypressFrame = keypressFrame + 1;
    keypressDataReady = 1;
    if(strcmp(keypressKey, 'downarrow') || strcmp(keypressKey, 'uparrow'))
        keypressKeyPrev = keypressKey;
    end
    keypressKey = event.Key;
    disp(keypressKey);
end