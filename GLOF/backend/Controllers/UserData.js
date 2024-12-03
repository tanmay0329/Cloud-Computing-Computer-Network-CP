const { GeneralPublicValidation,
    GeneralPublicLogIn,
    GovernmentAuthoritiesValidation,
    GovernmentAuthoritiesLogIn,
    EmergencyRespondersValidation,
    EmergencyRespondersLogIn } = require("../models/UserData");

async function handleGenralPublicLogIn(UserData) {
    try {
        const reuslt = await GeneralPublicLogIn.create(UserData);
        return reuslt;
    } catch (error) {
        console.log("error = ", error)
    }
}
async function handleGenralPublicValidation(UserData) {
    try {
        const reuslt = await GeneralPublicValidation.create(UserData);
        return reuslt;
    } catch (error) {
        console.log("error = ", error)
    }
}
async function handleGovernmentAuthLogin(UserData) {
    try {
        const reuslt = await GovernmentAuthoritiesLogIn.create(UserData);
        return reuslt;
    } catch (error) {
        console.log("error = ", error)
    }
}
async function handleGovernmentAuthValidation(UserData) {
    try {
        const reuslt = await GovernmentAuthoritiesValidation.create(UserData);
        return reuslt;
    } catch (error) {
        console.log("error = ", error)
    }
}
async function handleEmergencyRespLogIn(UserData) {
    try {
        const reuslt = await EmergencyRespondersLogIn.create(UserData);
        return reuslt;
    } catch (error) {
        console.log("error = ", error)
    }
}
async function handleEmergencyRespValidation(UserData) {
    try {
        const reuslt = await EmergencyRespondersValidation.create(UserData);
        return reuslt;
    } catch (error) {
        console.log("error = ", error)
    }
}

module.exports = {
    handleEmergencyRespLogIn,
    handleEmergencyRespValidation,
    handleGenralPublicLogIn,
    handleGenralPublicValidation,
    handleGovernmentAuthLogin,
    handleGovernmentAuthValidation,
}