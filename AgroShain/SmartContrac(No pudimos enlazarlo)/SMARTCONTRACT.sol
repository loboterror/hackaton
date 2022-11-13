pragma solidity ^0.8.7;

contract HOLAMUNDOSMARTCONTRACT {
    string private _mensaje;

    constructor(string memory mensaje){
        _mensaje = mensaje;

    }
    
    function getMensaje() public view returns (string memory){
        return _mensaje;
    }

    function setMensaje(string memory newMensaje) public {
        _mensaje = newMensaje;
    }

}


