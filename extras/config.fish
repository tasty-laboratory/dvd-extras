if status is-interactive
    # Commands to run in interactive sessions can go here
    set_color normal
    function fish_prompt
        echo 'λ '
    end
    alias 'l'='ls -lah'
    alias 'venv'='source ~/.venv/bin/activate.fish'
    alias 'sync'='source /home/arkin/.config/fish/config.fish'
    alias 'config'='vim /home/arkin/.config/fish/config.fish'
    
    export PX4_SIM_HOST_ADDR=[ipaddr]
end
