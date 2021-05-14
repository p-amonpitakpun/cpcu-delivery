const bcrypt = require('bcrypt');

let a = async() => {
    let x = await bcrypt.compare("}pR59qzgWW.L'+J{", "$2b$12$UEWvSVIl3Rrk/LznVVHC.eGNoqpqyQ/VZOky5Zzze9C0ffxfnF/ea");
    console.log(x);
}
a();