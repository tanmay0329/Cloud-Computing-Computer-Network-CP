const mongoose = require("mongoose")

const generalPublicLogIn = new mongoose.Schema({
    fullName:{
        type:String,
        required:true,
    },
    email:{
        type:String,
        required:true,
    },
    contact:{
        type:Number,
        required:true,
    },
    password:{
        type:String,
        required:true,
    }
}, {timestamps:true})

const GeneralPublicLogIn = mongoose.model("GeneralPublicLogInSchema",generalPublicLogIn);
const generalPublicValidation = new mongoose.Schema({
    location:{
        type:String,
        required:true,
    },
    preferredCommunication:{
        type:String,
        required:true,
    },
    governementIdProof:{
        type:String,
        required:true,
    },
    governementIdNumber:{
        type:String,
        required:true,
    }
}, {timestamps:true})

const GeneralPublicValidation = mongoose.model("GeneralPublicSignUpSchema",generalPublicValidation);

const governmentAuthoritiesLogIn = new mongoose.Schema({
    fullName:{
        type:String,
        required:true,
    },
    email:{
        type:String,
    },
    contact:{
        type:Number,
        required:true,
    },
    password:{
        type:String,
        required:true,
    },
    department:{
        type:String,
        required:true,
    },
},{timestamps:true})

const GovernmentAuthoritiesLogIn = mongoose.model("GovernmentAuthoritiesLogInSchema", governmentAuthoritiesLogIn)

const governmentAuthoritiesValidation = new mongoose.Schema({
    jurisdiction:{
        type:String,
    },
    emergencyContactInformation:{
        type:String,
        required:true,
    },
    accessLevel:{
        type:String,
    },
    governmentIdProof:{
        type:String,
        required:true,
    },
    governemntIdNumber:{
        type:String,
        required:true,
    },
},{timestamps:true})

const GovernmentAuthoritiesValidation = mongoose.model("GovernmentAuthoritiesSignUpSchema",governmentAuthoritiesValidation)



const emergencyRespondersLogIn = new mongoose.Schema({
    fullName:{
        type:String,
        required:true,
    },
    email:{
        type:String,
        
    },
    contact:{
        type:Number,
        required:true,
    },
    password:{
        type:String,
        required:true,
    },
},{timestamps:true})

const EmergencyRespondersLogIn = mongoose.model("EmergencyRespondersLogInSchema", emergencyRespondersLogIn);

const emergencyRespondersValidation = new mongoose.Schema({
    organization:{
        type:String,
        required:true,
    },
    areaofOperation:{
        type:String,
        required:true,
    },
    accessLevel:{
        type:String,
        required:true,
    },
    governmentIdProof:{
        type:String,
        required:true,
    },
    governemntIdNumber:{
        type:String,
        required:true,
    }
},{timestamps:true})

const EmergencyRespondersValidation = mongoose.model("EmergencyRespondersSignUpSchema", emergencyRespondersValidation);



module.exports = {GeneralPublicLogIn, 
    GeneralPublicValidation, 
    GovernmentAuthoritiesLogIn, 
    GovernmentAuthoritiesValidation,
    EmergencyRespondersLogIn, 
    EmergencyRespondersValidation}