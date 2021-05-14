const jwt = require('jsonwebtoken');

JWT_SECRET = "4fPZR{4Fw2Xaq'Yh";

const verifyJWT = async(token) => {
    try {
        return await jwt.verify(token, JWT_SECRET);
    } catch(err) {
        return false;
    }
}

module.exports = {
    authmiddleware: async(req, res, next) => {
        try {
            let token = req.headers.authorization.split("JWT ")[1];
            let verifyresult = await verifyJWT(token);
            if(!verifyresult)
                return res.sendStatus(401);
            req.user = verifyresult.username;
            next();
        } catch(err) {
            return res.sendStatus(401);
        }
    },
    getToken: (username)=>{
        return jwt.sign({
            exp: Math.floor(Date.now() / 1000) + (60 * 60 * 24),
            username
        }, JWT_SECRET);
    },
    verifyJWT
};