const mongoose = require("mongoose")

const alertDesignSchema = new mongoose.Schema({
    name:{
        type:String,
        required:true,
    },
    contactInfo:{
        type:Number,
        required:true,
    },
    email:{
        type:String,
        required:true,
    },
    alertMsg:{
        type:String,
        required:true
    }
}, {timestamps:true})

const alertSchema = mongoose.model("alertDesginSchema", alertDesignSchema);


module.exports = {alertSchema}