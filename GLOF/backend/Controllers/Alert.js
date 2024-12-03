const {
    alertSchema } = require("../models/Alerts")

async function handleAlertForm(...Data) {
    try {
        console.log("This is data - ",Data)
        const reuslt = await alertSchema.create(Data);
        return reuslt;
    } catch (error) {
        console.log("error = ", error)
    }
}

async function handleAllAlertForm() {
    try {
        const result = await alertSchema.find({});
        return result;
    } catch (error) {
        console.log("Error at getting all alert forms = ",error)
    }
}

module.exports = {
    handleAlertForm,
    handleAllAlertForm
}